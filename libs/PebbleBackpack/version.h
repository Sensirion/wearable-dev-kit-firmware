#ifndef VERSION_H
#define VERSION_H

#ifdef GIT_VERSION
// temporary macro to stringify VERSION
#define _STR(x) #x
#define STR(x) _STR(x)
static constexpr const char VERSION[] = STR(GIT_VERSION);
#undef STR
#undef _STR
#else
// if we don't build with make we mark the version as custom
static constexpr const char VERSION[] = "custom-build";
#endif

/**
 * Helper to get the length of a C-String. Will return the same as strlen() as
 * long as the input does not contain embedded '\0' example "foo'\0'bar"
 */
template<size_t N>
constexpr size_t c_string_length(char const (&)[N])
{
    return N-1;
}

static constexpr size_t VERSION_LENGTH = c_string_length(VERSION);

#endif
