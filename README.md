# ℹ️ PSMoveServiceEx
This fork has been created to expand the PSMoveSerivce lifespan with fixes, optimizations and new features.
Please use the original PSMoveService Wiki for now until i added my own.

# ℹ️ PSMoveService
A background service that manages multiple PSMove Controllers and PS3 Eye Cameras. Clients connect to the service and stream PSMove Controller state (position, orientation and button presses). A visual client front-end is used for controller/tracker configuration. The [FAQ](https://github.com/psmoveservice/PSMoveService/wiki/Frequently-Asked-Questions) is a good starting point for any specific questions you may have about the project. 

 # 🔗 PSMoveServiceEx with SteamVR
ℹ️ *Requires [PSMoveServiceEx - Virtual Device Manager](https://github.com/Timocop/PSMoveServiceEx-Virtual-Device-Manager/tree/master)*

[PSVR SteamVR](https://github.com/Timocop/PSMoveServiceEx/assets/22834512/70b5b729-4c59-4d9f-b554-23af9b494fdb)

# 📖 Documentation
* Use the [Wiki](https://github.com/Timocop/PSMoveServiceEx/wiki) for guidance and tutorials. **(WORK IN PROGRESS)**
* Old general setup legacy guides, troubleshooting and design docs can be found on the [wiki](https://github.com/psmoveservice/PSMoveService/wiki)

# ❓ Getting Help
Please start with the wiki. If you can't find help with your problem then please search through the issues (especially the closed ones) to see if your problem has been addressed already. 
If you still find no previous mention of your problem then you have these options:
- Join the official [PSMoveServiceEx Discord Server](https://discord.gg/mSk9ua7exg).
- Use the [GitHub Discussions](https://github.com/Timocop/PSMoveServiceEx/discussions).
- Create an issue if you think you found a bug or if you have a problem with the program.
  - *⚠️ The ``Issues`` page should not be used for regular questions!*

# 💙 Attribution and Thanks
Special thanks to the following people who helped make this project possible:
* HipsterSloth for maintaining PSMoveService for so long ♥️
* Thomas Perl and his [psmoveapi](https://github.com/thp/psmoveapi) project which laid the groundwork for this project.
* Alexander Nitsch and his work on [psmove-pair-win](https://github.com/nitsch/psmove-pair-win) that enabled psmove controller pairing over Bluetooth on Windows. Alex also did nearly all of the investigation into the PSMove communication protocol.
* Eugene Zatepyakin and his [PS3EYEDriver](https://github.com/inspirit/PS3EYEDriver) project which enabled access to the PS3Eye camera.
* Ritesh Oedayrajsingh Varma and his work on [PS3EYEDriver](https://github.com/rovarma/PS3EYEDriver) enabling improved camera polling performance (consistent 60fps)
* Frédéric Lopez and his work on [PS-Move-Setup](https://github.com/Fredz66/PS-Move-Setup) that enabled co registration of  and HMD with the PSMove.
* Greg New - Improvements to the SteamVR plugin and config tool
* YossiMH - Improvements to touch pad mappings and help with the HMD/Controller alignment tool
* William (zelmon64) - Many improvements to config tool UX, beta testing, and troubleshooting hero
* Antonio Jose Ramos Marquez - Work on PS4EyeDriver and PSX hardware reverse engineering
