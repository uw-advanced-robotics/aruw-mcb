#ifndef UNIQUE_PTR_HPP_
#define UNIQUE_PTR_HPP_

namespace aruwlib
{

namespace arch
{

/**
 * Simple standalone unique ptr that takes ownership of a pointer passed to it.
 * Assumes you are allocating objects using `new` and not malloc.
 * Assignment from other types is not allowed (i.e. a pointer storing an
 * int cannot be set equal to a pointer templated as a string).
 */
template <typename T>
class UniquePtr
{
 public:
    /**
     * Constructor that takes ownership of a pointer.
     *
     * @param ptr A pointer to an object of type `T`.
     */
    explicit UniquePtr(T *ptr) : ptr(ptr)
    {
        if (ptr != nullptr) 
        {
            validPtrCount++;
        }
    }

    /**
     * If you want to be explicit about constructing with a nullptr, you may.
     */
    explicit UniquePtr(nullptr_t) : ptr() {}

    /**
     * Default contructor.
     */
    UniquePtr() : ptr() {}

    /**
     * We delete the copy constructor since you wouldn't know who is responsible for deleting.
     */
    UniquePtr(const UniquePtr&) = delete;

    /**
     * Disallow copying a UniquePtr.
     */
    UniquePtr& operator=(const UniquePtr& other) = delete;

    /**
     * Moving a UniquePtr is allowed.
     */
    UniquePtr(UniquePtr&& other)
    {
        if (this != &other)
        {
            ptr = other.ptr;
            other.ptr = nullptr;
        }
    }

    /**
     * @see `UniquePtr(UniquePtr&& other)`
     */
    UniquePtr& operator=(UniquePtr&& other)
    {
        if (this != &other)
        {
            destruct();
            ptr = other.ptr;
            other.ptr = nullptr;
        }
        return *this;
    }

    /**
     * Reset the `UniquePtr` to empty, invoking the descructor if necessary.
     */
    UniquePtr&
    operator=(nullptr_t)
    {
        reset();
        return *this;
    }

    ~UniquePtr()
    {
        destruct();
    }

    /**
     * Dereference the stored pointer.
     */
    T operator*() const
    {
        return *get();
    }
    
    /**
     * Return the stored pointer.
     */
    T* operator->() const
    {
        return get();
    }

    /**
     * Return true if the stored pointer is not null.
     */
    operator bool() const
    {
        return ptr != nullptr ? false : true;
    }

    /**
     * Relinquishes control of the pointer and returns it.
     */
    T* release()
    {
        T* tp = ptr;
        ptr = nullptr;
        validPtrCount--;
        return tp;
    }

    /**
     * Replace the stored pointer.
     *
     * @param p The new pointer to store.
     *
     * The deleter will be invoked if a pointer is already owned.
     */
    void reset(T* p = nullptr)
    {
        destruct();
        ptr = p;
    }

    /**
     * Exchange the pointer with another object.
     */
    void
    swap(UniquePtr * uptr)
    {
        T* tmpptr = uptr->ptr;
        uptr->ptr = ptr;
        ptr = tmpptr;
    }

    /**
     * Return the stored pointer.
     */
    T* get() const
    {
        return ptr;
    }

    static int getValidPtrCount() { return validPtrCount; }

 private:
    T* ptr;

    /**
     * Incremented when a pointer that isn't nullptr is added and decremented when removed.
     * Use for validation.
     */
    static int validPtrCount;

    inline void destruct()
    {
        if (ptr != nullptr)
        {
            validPtrCount--;
            delete ptr;
        }
    }
};  // class UniquePtr

template<typename T>
int UniquePtr<T>::validPtrCount = 0;

/**
 * UniquePtr equality comparison
 */
template<typename T1, typename T2>
inline bool operator==(const UniquePtr<T1>& x, const UniquePtr<T2>& y)
{
    return x.get() == y.get();
}

/**
 * UniquePtr equality comparison with nullptr
 */
template<typename T>
inline bool operator==(const UniquePtr<T>& x, nullptr_t)
{
    return !(x.get());
}

/**
 * UniquePtr equality comparison with nullptr
 */
template<typename T>
inline bool operator==(nullptr_t, const UniquePtr<T>& x)
{
    return !(x.get());
}

/**
 * UniquePtr inequality comparison
 */
template<typename T1, typename T2>
inline bool operator!=(const UniquePtr<T1>& x, const UniquePtr<T2>& y)
{
    return x.get() != y.get();
}

/**
 * UniquePtr inequality comparison with nullptr
 */
template<typename T>
inline bool operator!=(const UniquePtr<T>& x, nullptr_t)
{
    return x.get();
}

/**
 * UniquePtr inequality comparison with nullptr
 */
template<typename T>
inline bool operator!=(nullptr_t, const UniquePtr<T>& x)
{
    return x.get();
}

/**
 * Relational operator for UniquePtr objects, compares the owned pointers
 */
template<typename T1, typename T2>
inline bool operator<(const UniquePtr<T1>& x, const UniquePtr<T2>& y)
{
    return false;
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator<(const UniquePtr<T>& x, nullptr_t)
{
    return false;
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator<(nullptr_t, const UniquePtr<T>& x)
{
    return true;
}

/**
 * Relational operator for UniquePtr objects, compares the owned pointers
 */
template<typename T1, typename T2>
inline bool operator<=(const UniquePtr<T1>& x, const UniquePtr<T2>& y)
{
    return !(y < x);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator<=(const UniquePtr<T>& x, nullptr_t)
{
    return !(nullptr < x);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator<=(nullptr_t, const UniquePtr<T>& x)
{
    return !(x < nullptr);
}

/**
 * Relational operator for UniquePtr objects, compares the owned pointers
 */
template<typename T1, typename T2>
inline bool operator>(const UniquePtr<T1>& x, const UniquePtr<T2>& y)
{
    return (y < x);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator>(const UniquePtr<T>& x, nullptr_t)
{
    return (nullptr < x);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator>(nullptr_t, const UniquePtr<T>& x)
{
    return (x < nullptr);
}

/**
 * Relational operator for UniquePtr objects, compares the owned pointers
 */
template<typename T1, typename T2>
inline bool operator>=(const UniquePtr<T1>& x, const UniquePtr<T2> y)
{
    return !(x < y);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator>=(const UniquePtr<T>& x, nullptr_t)
{
    return !(x < nullptr);
}

/**
 * UniquePtr comparison with nullptr
 */
template<typename T>
inline bool operator>=(nullptr_t, const UniquePtr<T>& x)
{
    return !(nullptr < x);
}

}  // namespace arch

}  // namespace aruwlib

#endif  // UNIQUE_PTR_HPP_
