#include "../psmoveservice/Device/Enumerator/GenericWebcamEnumerator.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    int failure_count = 0;

    void expect(bool condition, const char *description)
    {
        if (!condition)
        {
            ++failure_count;
            std::cerr << "FAILED: " << description << std::endl;
        }
    }
}

int main()
{
    const std::wstring symbolic_link =
        L"\\\\?\\usb#vid_1234&pid_abcd#camera-instance";
    const std::string stable_id =
        GenericWebcamEnumerator::makeStableId(symbolic_link);
    const std::string repeated_id =
        GenericWebcamEnumerator::makeStableId(symbolic_link);
    const std::string case_changed_id =
        GenericWebcamEnumerator::makeStableId(
            L"\\\\?\\USB#VID_1234&PID_ABCD#CAMERA-INSTANCE");
    const std::string different_id =
        GenericWebcamEnumerator::makeStableId(
            L"\\\\?\\usb#vid_1234&pid_abcd#other-instance");

    expect(stable_id == repeated_id, "stable ID is deterministic");
    expect(
        stable_id == case_changed_id,
        "symbolic-link identity is case-insensitive");
    expect(stable_id != different_id, "different symbolic links remain distinct");
    expect(stable_id.size() == 36, "stable ID has a fixed compact length");
    expect(stable_id.substr(0, 4) == "wmf_", "stable ID is namespaced");

    // This is the safety property that prevents disabled generic webcams from
    // being activated merely because tracker enumeration ran.
    const std::vector<std::string> no_enabled_devices;
    GenericWebcamEnumerator disabled_enumerator(no_enabled_devices, true);
    expect(!disabled_enumerator.is_valid(), "empty allowlist has no devices");
    expect(
        disabled_enumerator.get_last_error().empty(),
        "empty allowlist performs no platform discovery");

    if (failure_count == 0)
    {
        std::cout << "Generic webcam enumerator helper tests passed."
                  << std::endl;
        return EXIT_SUCCESS;
    }

    std::cerr << failure_count << " helper test(s) failed." << std::endl;
    return EXIT_FAILURE;
}
