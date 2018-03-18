#ifndef FUNCTION_HELPER_H
#define FUNCTION_HELPER_H

#ifdef OPTION_CLASS
//#define FUNCTION_DEF(class, name, type, ...) type class # :: # name(__VA_ARGS__)
#define FUNCTION_DECL(name, type, ...) type name(__VA_ARGS__)
#endif

#endif  // FUNCTION_HELPER_H