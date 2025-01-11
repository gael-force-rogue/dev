This is my main development repository.

### Folders

- `src` is the main Vex V5 project
- `vexcom` is a binary that the VexCode Extension seems to use to upload
  binaries to the brain. I'll explore it more once I figure out how to build the
  project from the command line.
- `old` archived code

### VPP

This is a little wrapper library around VEXCode to improve the API. I don't
suggest using it for now but if you do please credit me.

### Hierarchy

1. VEX (vex.h)
2. VPP Essentials & Helpers (helpers.h)
3. VPP Library
4. User (config.h)
5. Rest of program files
