
// #define CATCH_CONFIG_PREFIX_ALL

// #include <algorithm>
// #include <iterator>

// #include "perceive/contrib/catch.hpp"
// #include "perceive/utils/shifty-queue.hpp"

// // [1] All vector functionality,
// // [2] Shifty vector functionality for fast queues
// // [3] Short vector optimization -- see Chandler's talk
// // [4] Template parameter: move invalidates iterators
// // [5] Template parameter: inplace storage (i.e., no resize)
// // [6] No exceptions

// namespace perceive
// {
// template<typename T, typename Allocator = std::allocator<T>>
// class VectorBits : public Allocator
// {
//  public:
//    // ## ------------------------------------------------------- types
//    using value_type             = T;
//    using reference              = value_type&;
//    using const_reference        = const value_type&;
//    using allocator_type         = Allocator;
//    using size_type              = std::size_t;
//    using difference_type        = std::ptrdiff_t;
//    using pointer                = T*;
//    using const_pointer          = const T*;
//    using iterator               = pointer; // does this bork some allocators?
//    using const_iterator         = const_pointer;
//    using reverse_iterator       = std::reverse_iterator<iterator>;
//    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
//    using alloc_traits           = std::allocator_traits<Allocator>;

//    struct Ptrs
//    {
//       pointer start; // begin storage, capacity passed separately
//       pointer begin; // begin pointer of vector
//       pointer end;   // end point of vector
//    };

//    // ## ------------------------------------------------------ traits
//    static constexpr bool is_nothrow_move_copy_assign
//        = std::is_nothrow_move_constructible<T>::value
//          || std::is_nothrow_copy_constructible<T>::value;

//    static constexpr bool is_nothrow_shiftable
//        = std::is_trivially_copyable<T>::value || is_nothrow_move_copy_assign;

//    static constexpr bool is_nothrow_copyable
//        = std::is_trivially_copyable<T>::value
//          || std::is_nothrow_copy_constructible<T>::value;

//    static constexpr bool is_shiftable = is_nothrow_shiftable
//                                         ||
//                                         std::is_move_constructible<T>::value
//                                         ||
//                                         std::is_copy_constructible<T>::value;

//    // ## ------------------------------------------------ construction
//    explicit VectorBits(pointer storage, size_type sz)
//        : Allocator()
//    {
//       init_sm_(calc_offset_(storage), sz);
//       assert(storage == bgn_store());
//       assert(sz == capacity());
//    }

//    explicit VectorBits(pointer storage, size_type sz, const Allocator& alloc)
//        : Allocator(alloc)
//    {
//       init_sm_(calc_offset_(storage), sz);
//       assert(storage == bgn_store());
//       assert(sz == capacity());
//    }

//    explicit VectorBits(Ptrs* ptrs, size_type sz)
//        : Allocator()
//    {
//       init_bg_(calc_offset_(ptrs), sz);
//    }

//    explicit VectorBits(Ptrs* ptrs, size_type sz, const Allocator& alloc)
//        : Allocator(alloc)
//    {
//       init_bg_(calc_offset_(ptrs), sz);
//    }

//    size_type capacity() const noexcept { return is_sm() ? sm_cap() :
//    bg_cap(); } size_type size() const noexcept { return cend() - cbegin(); }

//    bool is_sm() const noexcept { return !(bits_ & flag_bit); }
//    int8_t offset() const noexcept { return (bits_ & offset_mask) >> 48; }

//    void set_begin(pointer begin) noexcept
//    {
//       if(is_sm())
//          set_sm_begin(begin - bgn_store());
//       else
//          ptrs_()->begin = begin;
//    }

//    void set_end(pointer end) noexcept
//    {
//       if(is_sm())
//          set_sm_end(end - bgn_store());
//       else
//          ptrs_()->end = end;
//    }

//    const_pointer cbgn_store() const noexcept
//    {
//       return is_sm() ? reinterpret_cast<const_pointer>(cstorage_())
//                      : cptrs_()->start;
//    }
//    const_pointer cend_store() const noexcept
//    {
//       return cbgn_store() + capacity();
//    }
//    const_pointer cbegin() const noexcept
//    {
//       return is_sm() ? (cbgn_store() + sm_begin()) : cptrs_()->begin;
//    }
//    const_pointer cend() const noexcept
//    {
//       return is_sm() ? (cbgn_store() + sm_end()) : cptrs_()->end;
//    }

//    pointer bgn_store() noexcept { return const_cast<pointer>(cbgn_store()); }
//    pointer end_store() noexcept { return const_cast<pointer>(cend_store()); }
//    pointer begin() noexcept { return const_cast<pointer>(cbegin()); }
//    pointer end() noexcept { return const_cast<pointer>(cend()); }

//    size_type max_size() const noexcept { return cap_mask; }

//    std::string to_str()
//    {
//       using namespace perceive;

//       auto values = [&]() {
//          bool comma = false;
//          std::stringstream ss{""};
//          std::for_each(begin(), end(), [&](const_reference o) {
//             if(comma) { ss << ", "; }
//             ss << o;
//             comma = true;
//          });
//          return ss.str();
//       };

//       return format(
//           R"V0G0N(
// {
//    &[0]          = %p + %d = %p
//    max-size      = 0x%016x
//    bits          = 0x%016x
//    is-sm         = %s
//    sizeof(T)     = %d
//    size          = %d
//    indices       = %d .. [%d..%d) .. %d
//    values        = [%s]
// }
// )V0G0N",
//           this,
//           offset(),
//           cstorage_(),
//           max_size(),
//           bits_,
//           str(is_sm()),
//           sizeof(T),
//           size(),
//           0,
//           int(begin() - bgn_store()),
//           int(end() - bgn_store()),
//           capacity(),
//           values());
//    }

//  private:
//    using bits_type = uint64_t;
//    using ptr_ptr   = pointer*;

//    static constexpr bits_type flag_bit    = bits_type(0x8000000000000000ull);
//    static constexpr bits_type offset_mask = bits_type(0x00ff000000000000ull);
//    static constexpr bits_type cap_mask    = bits_type(0x0000ffffffffffffull);
//    static constexpr bits_type sm_cap_mask = bits_type(0x0000ffff00000000ull);
//    static constexpr bits_type sm_bgn_mask = bits_type(0x00000000ffff0000ull);
//    static constexpr bits_type sm_end_mask = bits_type(0x000000000000ffffull);

//    bits_type bits_ = 0;

//    const Ptrs* cptrs_() const noexcept
//    {
//       assert(!is_sm());
//       return reinterpret_cast<const Ptrs*>(cstorage_());
//    }

//    Ptrs* ptrs_() noexcept { return const_cast<Ptrs*>(cptrs_()); }

//    const char* cstorage_() const noexcept
//    {
//       return reinterpret_cast<const char*>(this) + offset();
//    }

//    template<typename P> int8_t calc_offset_(const P* storage) const noexcept
//    {
//       auto ptr_offset = reinterpret_cast<const char*>(storage)
//                         - reinterpret_cast<const char*>(this);
//       assert(ptr_offset >= std::numeric_limits<int8_t>::lowest());
//       assert(ptr_offset < std::numeric_limits<int8_t>::max());
//       return static_cast<int8_t>(ptr_offset);
//    }

//    void init_sm_(int8_t ptr_offset, size_type capacity) noexcept
//    {
//       bits_ = ((bits_type(ptr_offset) & 0x00ffull) << 48)
//               | ((bits_type(capacity) & 0xffffull) << 32)
//               | ((bits_type(0) & 0xffffull) << 16)
//               | ((bits_type(0) & 0xffffull) << 0);
//       assert(is_sm());
//       assert(capacity == size_type(sm_cap()));
//    }

//    void init_bg_(int8_t ptr_offset, size_type capacity) noexcept
//    {
//       bits_ = flag_bit | ((bits_type(ptr_offset) & 0x00ffull) << 48)
//               | ((bits_type(capacity) & cap_mask) << 0);
//       assert(!is_sm());
//       assert(this->capacity() == capacity);
//    }

//    size_type bg_cap() const noexcept { return (bits_ & cap_mask); }
//    int sm_cap() const noexcept { return (bits_ & sm_cap_mask) >> 32; }
//    int sm_begin() const noexcept { return (bits_ & sm_bgn_mask) >> 16; }
//    int sm_end() const noexcept { return (bits_ & sm_end_mask) >> 0; }

//    void set_sm_begin(int begin) noexcept
//    {
//       assert(is_sm());
//       bits_ = (bits_ & ~sm_bgn_mask) | ((bits_type(begin) & 0xffffull) <<
//       16); assert(sm_begin() == begin);
//    }

//    void set_sm_end(int end) noexcept
//    {
//       assert(is_sm());
//       bits_ = (bits_ & ~sm_end_mask) | ((bits_type(end) & 0xffffull) << 0);
//       assert(sm_end() == end);
//    }

// }; // namespace perceive

// CATCH_TEST_CASE("ShiftyQueue", "[shifty_queue]")
// {
//    CATCH_SECTION("shifty-queue")
//    {
//       // int buffer[5] = {1, 2, 3, 4, 5};
//       // {
//       //    VectorBits<int> Q(buffer, 5);
//       //    Q.set_begin(&buffer[1]);
//       //    Q.set_end(&buffer[5]);
//       //    cout << Q.to_str() << endl;
//       // }

//       // {
//       //    VectorBits<int>::Ptrs p;
//       //    p.start = &buffer[0];
//       //    p.begin = &buffer[1];
//       //    p.end   = &buffer[5];
//       //    VectorBits<int> R(&p, 5);
//       //    cout << R.to_str() << endl;
//       // }

//       //
//       if(false) {
//          // shifty_queue<int> Q;

//          // Q.stream(cout);

//          // Q.push_back(10);
//          // Q.stream(cout);

//          // INFO(format("sizeof Q        = {}", sizeof(Q)));
//          // INFO(format("sizeof impl     = {}", Q.sizeof_impl()));
//          // INFO(format("sizeof packed   = {}", Q.sizeof_impl_k()));
//          // INFO(format("sizeof pointers = {}", Q.sizeof_impl_p()));
//       }
//    }
// }

// } // namespace perceive
