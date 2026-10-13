
#pragma once

#include <chrono>
#include <condition_variable>
#include <optional>

namespace perceive::pipeline::realtime
{
// ---------------------------------------------------------------- BoundedQueue
// A FIFO of at most `capacity` elements, shared between threads. The waits are
// plain condition variables: a thread that waits here never runs a job of the
// global thread pool. `push` blocks while the queue is full, unless the queue
// drops its oldest element to make room (`drop_oldest`). `close()` wakes every
// waiter; after it `push` fails, and `pop` drains what is left. The storage is
// a ring allocated in the constructor, so pushing never allocates.
//
template<typename T> class BoundedQueue
{
 private:
   mutable std::mutex padlock_;
   mutable std::condition_variable not_empty_;
   mutable std::condition_variable not_full_;
   vector<std::optional<T>> ring_; // `capacity_` slots
   size_t capacity_    = 1;
   size_t head_        = 0; // index of the oldest element
   size_t size_        = 0;
   bool drop_oldest_   = false;
   bool is_closed_     = false;
   uint64_t n_pushed_  = 0;
   uint64_t n_dropped_ = 0;

   void push_locked_(T&& value);
   T pop_locked_();

 public:
   explicit BoundedQueue(size_t capacity, bool drop_oldest = false);
   BoundedQueue(const BoundedQueue&)            = delete;
   BoundedQueue(BoundedQueue&&)                 = delete;
   ~BoundedQueue()                              = default;
   BoundedQueue& operator=(const BoundedQueue&) = delete;
   BoundedQueue& operator=(BoundedQueue&&)      = delete;

   size_t size() const noexcept;
   size_t capacity() const noexcept { return capacity_; }
   bool drop_oldest() const noexcept { return drop_oldest_; }
   bool is_closed() const noexcept;
   uint64_t n_pushed() const noexcept;  // lifetime count of accepted elements
   uint64_t n_dropped() const noexcept; // oldest elements dropped for room
   bool has_room(size_t n) const noexcept; // size() + n <= capacity()

   // Blocks while full, unless `drop_oldest`. Returns false after `close()`.
   bool push(T value);

   // Never blocks. Returns false when full (and not `drop_oldest`), or closed.
   bool try_push(T value) noexcept;

   // Blocks until there is an element. `nullopt` once closed and empty.
   std::optional<T> pop();

   // As `pop`, but returns `nullopt` after `timeout`.
   std::optional<T> pop_for(std::chrono::milliseconds timeout);

   // Never blocks.
   std::optional<T> try_pop() noexcept;

   // Blocks until `has_room(n)`. Returns false if the queue is closed first.
   bool wait_for_room(size_t n) const;

   void close() noexcept;
};

// ---------------------------------------------------------------- Construction
//
template<typename T>
BoundedQueue<T>::BoundedQueue(size_t capacity, bool drop_oldest)
    : capacity_(capacity)
    , drop_oldest_(drop_oldest)
{
   Expects(capacity_ > 0);
   ring_.resize(capacity_);
}

// ------------------------------------------------------------------------ size
//
template<typename T> size_t BoundedQueue<T>::size() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return size_;
}

// ------------------------------------------------------------------- is-closed
//
template<typename T> bool BoundedQueue<T>::is_closed() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return is_closed_;
}

// -------------------------------------------------------------------- n-pushed
//
template<typename T> uint64_t BoundedQueue<T>::n_pushed() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_pushed_;
}

// ------------------------------------------------------------------- n-dropped
//
template<typename T> uint64_t BoundedQueue<T>::n_dropped() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_dropped_;
}

// -------------------------------------------------------------------- has-room
//
template<typename T> bool BoundedQueue<T>::has_room(size_t n) const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return size_ + n <= capacity_;
}

// ----------------------------------------------------------------- push-locked
// The caller holds `padlock_`, and there is room or `drop_oldest_` is set.
//
template<typename T> void BoundedQueue<T>::push_locked_(T&& value)
{
   if(size_ == capacity_) {
      pop_locked_();
      ++n_dropped_;
   }
   ring_[(head_ + size_) % capacity_] = std::move(value);
   ++size_;
   ++n_pushed_;
}

// ------------------------------------------------------------------ pop-locked
// The caller holds `padlock_`, and the queue is not empty.
//
template<typename T> T BoundedQueue<T>::pop_locked_()
{
   auto& slot = ring_[head_];
   T value    = std::move(*slot);
   slot.reset();
   head_ = (head_ + 1) % capacity_;
   --size_;
   return value;
}

// ------------------------------------------------------------------------ push
//
template<typename T> bool BoundedQueue<T>::push(T value)
{
   {
      std::unique_lock<std::mutex> lock(padlock_);
      if(!drop_oldest_)
         not_full_.wait(lock,
                        [this]() { return is_closed_ || size_ < capacity_; });
      if(is_closed_) return false;
      push_locked_(std::move(value));
   }
   not_empty_.notify_one();
   return true;
}

// -------------------------------------------------------------------- try-push
//
template<typename T> bool BoundedQueue<T>::try_push(T value) noexcept
{
   {
      std::lock_guard<std::mutex> lock(padlock_);
      if(is_closed_) return false;
      if(!drop_oldest_ && size_ >= capacity_) return false;
      push_locked_(std::move(value));
   }
   not_empty_.notify_one();
   return true;
}

// ------------------------------------------------------------------------- pop
//
template<typename T> std::optional<T> BoundedQueue<T>::pop()
{
   std::optional<T> ret;
   {
      std::unique_lock<std::mutex> lock(padlock_);
      not_empty_.wait(lock, [this]() { return is_closed_ || size_ > 0; });
      if(size_ == 0) return ret;
      ret = pop_locked_();
   }
   not_full_.notify_all();
   return ret;
}

// --------------------------------------------------------------------- pop-for
//
template<typename T>
std::optional<T> BoundedQueue<T>::pop_for(std::chrono::milliseconds timeout)
{
   std::optional<T> ret;
   {
      std::unique_lock<std::mutex> lock(padlock_);
      not_empty_.wait_for(
          lock, timeout, [this]() { return is_closed_ || size_ > 0; });
      if(size_ == 0) return ret;
      ret = pop_locked_();
   }
   not_full_.notify_all();
   return ret;
}

// --------------------------------------------------------------------- try-pop
//
template<typename T> std::optional<T> BoundedQueue<T>::try_pop() noexcept
{
   std::optional<T> ret;
   {
      std::lock_guard<std::mutex> lock(padlock_);
      if(size_ == 0) return ret;
      ret = pop_locked_();
   }
   not_full_.notify_all();
   return ret;
}

// --------------------------------------------------------------- wait-for-room
//
template<typename T> bool BoundedQueue<T>::wait_for_room(size_t n) const
{
   std::unique_lock<std::mutex> lock(padlock_);
   not_full_.wait(lock,
                  [this, n]() { return is_closed_ || size_ + n <= capacity_; });
   return !is_closed_;
}

// ----------------------------------------------------------------------- close
//
template<typename T> void BoundedQueue<T>::close() noexcept
{
   {
      std::lock_guard<std::mutex> lock(padlock_);
      is_closed_ = true;
   }
   not_empty_.notify_all();
   not_full_.notify_all();
}

} // namespace perceive::pipeline::realtime
