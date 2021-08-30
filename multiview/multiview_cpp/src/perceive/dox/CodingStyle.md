
Coding Style                {#coding_style}
============


## Notes from Code Review ##

 * Always initialize data, unless there's a very very special reason not to.
 * Always prefer references to pointers.
 * It's okay to copy (most) data structures. Do so if it make semantics simpler.


## Naming conventions ##

 * Type names start with a capital letter.
 * All other names are in snake_case.
 * Private members and methods end with an underscore.

## Plain Old Data Types (PODs) ##

 * PODs should be `struct` (as opposed to `class`).
 * Members are in snake_case.

```
struct Point
{
   int x = 0;
   int y = 0;
};
```

## Classes ##

 * All members are private.
 * All private members and methods end with an underscore.
 * Place member definitions at the top.
 * Then place constructors/assignment.
 * Then place Getters/Setters
 * Then place actions

```
class Widget
{
private:
   class Pimpl;
   unique_ptr<Pimpl> pimpl_ = nullptr;

public:
   // Construction/Assignment
   Widget();
   Widget(const Widget&) = delete;
   Widget(Widget&&) = default;
   ~Widget();
   Widget operator=(const Widget&) = delete;
   Widget operator=(Widget&&) = default;

   // Getters/Setters
   int left() const;
   void set_left(int l) const;

   // Actions
   void invalidate();
};
```

## Rules to be Considered ##

 * Except for non-private inheritance, one header file should rarely include another.
 * Use 'Excepts', 'Ensures' for pre/post conditions, and 'assert' in the middle. We will transition to the `contracts` with C++20. (https://herbsutter.com/category/c/)

## Never use C-style Casts ##

```
int * val = (int *) ptr; // wrong
int * val = static_cast<int *>(ptr); // right
```


## Make Functions `static` Whenever Possible ##

Every function is either `static` or listed in a header file.

```
void some_function(...);        // wrong, unless this appears in hpp file.
static void some_function(...); // correct

```

Some of the hardest bugs to track down are caused by violations of the ODR. The bug may not even show up until some seemingly random change to the build (e.g., adding an unrelated file) causes the linker to switch between function definitions, and then: _*kBAM!*_.

## Always put banners above functions ##

Functions should have an 80 character banner one space above them, like so:

```
// ------------------------------------------------------------ my cool function

static void my_cool_function(...)
{
    // ...
}

```

## Make most variables const ##

```
int value = 0; // wrong
const int value = 0; // it's verbose, but prefer immutable variables
```


