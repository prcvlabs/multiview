
#include "classifier-registry.hpp"

#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/spin-lock.hpp"

namespace perceive
{
struct ClassifierLookup
{
 private:
   mutable SpinLock padlock;
   std::unordered_map<string, unique_ptr<Classifier>> classifiers;
   std::unordered_map<string, shared_ptr<SpinLock>> loading_spinlocks;

   // If TRUE, we successfully set to 'loading', and must attempt to load
   // If FALSE, then some other thread is 'loading', and we must wait
   shared_ptr<SpinLock> get_loading_lock(const string& key) noexcept
   {
      shared_ptr<SpinLock> out = nullptr;
      {
         lock_guard lock(padlock);

         { // Check to see with someone sneakily stashed the classifier
            auto ii = classifiers.find(key);            // while we weren't
            if(ii != cend(classifiers)) return nullptr; // looking
         }

         auto ii = loading_spinlocks.find(key);
         if(ii == cend(loading_spinlocks)) {
            auto loading_lock = make_shared<SpinLock>();
            loading_lock->lock();
            loading_spinlocks[key] = std::move(loading_lock);
         } else {
            out = ii->second;
         }
      }

      if(out == nullptr) { // here we do the load...
         auto classifier = make_unique<Classifier>();
         try {
            fetch(*classifier, key);
         } catch(std::exception& e) {
            LOG_ERR(
                format("exception loading classifier '{}': {}", key, e.what()));
            classifier.reset();
         }

         { // Finish up...
            lock_guard lock(padlock);
            Expects(classifiers.find(key) == cend(classifiers));
            classifiers[key] = std::move(classifier);

            auto ii = loading_spinlocks.find(key);
            Expects(ii != cend(loading_spinlocks));
            Expects(ii->second != nullptr);
            ii->second->unlock();
            loading_spinlocks.erase(ii);
         }
      }

      return out;
   }

   // Get's without attempting to load/modify
   const Classifier* get_(const string& key) const noexcept
   {
      lock_guard lock(padlock);
      auto ii = classifiers.find(key);
      return ii == cend(classifiers) ? nullptr : ii->second.get();
   }

 public:
   const Classifier* get(string_view key_sv) noexcept
   {
      string key(cbegin(key_sv), end(key_sv));
      auto ret = get_(key);
      if(ret != nullptr) return ret; // winners!

      shared_ptr<SpinLock> loading_lock = get_loading_lock(key);
      if(loading_lock != nullptr)
         lock_guard<SpinLock> lock(*loading_lock); // wait for other thread

      return get_(key); // If nullptr, then classifier couldn't be loaded
   }
};

const Classifier* get_classifier(string_view key) noexcept
{
   static ClassifierLookup classifiers;
   return classifiers.get(key);
}

} // namespace perceive
