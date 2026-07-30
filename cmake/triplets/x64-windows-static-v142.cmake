set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE static)
set(VCPKG_LIBRARY_LINKAGE static)

# Boost 1.83's vcpkg helper predates the MSVC 14.44 (vc144) library tag.
# The Windows 2022 runner retains v142, which that helper supports.
set(VCPKG_PLATFORM_TOOLSET v142)
