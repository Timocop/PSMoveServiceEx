#ifndef PSMOVE_SERVICE_H
#define PSMOVE_SERVICE_H

//-- includes -----
#include <string>

//-- definitions -----
class PSMoveService
{
public:
    struct ProgramSettings
    {
        std::string log_level;
        std::string admin_password;
		std::string working_directory;
		bool disable_morpheus;
		int minimum_virtual_hmd_count;
		int minimum_virtual_controller_count;
		int minimum_virtual_tracker_count;
    };

    PSMoveService();
    virtual ~PSMoveService();

    static PSMoveService *getInstance()
    { return m_instance; }
    const ProgramSettings *getProgramSettings() const
    { return &m_settings; }

    int exec(int argc, char** argv);

private:
    // Global program settings
    ProgramSettings m_settings;

    // Singleton instance of the class
    static PSMoveService *m_instance;
};

#endif // PSMOVE_SERVICE_H
