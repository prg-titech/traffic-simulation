// Option Standard: Use class-based implementation and render output.

#define OPTION_STANDARD 1
#define PTR(class) class*
#define CPTR(class) class* const
#define THIS this
#define METHOD_CALL(class, ptr, name, ...) (ptr)->name(__VA_ARGS__)
#define DEVICE_MODE
