# FTC Development Environment

## 🚀 **Quick Start**

```bash
# Initialize the development environment (one-time setup)
./init.sh

# Build your APK
./build.sh
```

## 🎯 **Clean Architecture**

- **`ftcSDK/`** - OOTB (Out-of-the-Box) SDK from official repository
- **`teamCode/`** - Your work directory (symlinked to ftcSDK)
- **`output/`** - Build outputs (APK, logs, etc.)

## 📁 **Project Structure**

```
android/
├── ftcSDK/                    # OOTB SDK (git cloned)
├── teamCode/                  # Your work (symlinked to ftcSDK)
│   └── src/main/java/org/firstinspires/ftc/teamcode/error404/
│       └── (your OpModes here)
├── output/                    # Build outputs
│   └── TeamCode-debug.apk
├── init.sh                    # 🚀 Initialization script
└── build.sh                   # 🔨 Build script
```

## ✨ **Benefits**

- ✅ **Minimal setup**: One command to initialize
- ✅ **OOTB SDK**: Always up-to-date with official repository
- ✅ **Clean separation**: Your code in teamCode, SDK in ftcSDK
- ✅ **Symlinked**: Edit in teamCode, build in ftcSDK
- ✅ **No boilerplate**: No complex gradle configurations

## 🔄 **Workflow**

1. **Initialize**: `./init.sh` (clones ftcSDK, creates symlinks)
2. **Develop**: Edit your OpModes in `teamCode/src/main/java/org/firstinspires/ftc/teamcode/error404/`
3. **Build**: `./build.sh` (builds using ftcSDK)
4. **Deploy**: Install `output/TeamCode-debug.apk` on Control Hub

## 🔧 **How It Works**

- **init.sh**: Clones ftcSDK if needed, creates symlinks, sets up build scripts
- **build.sh**: Builds using ftcSDK directly (maximum compatibility)
- **Symlinks**: Your teamCode is symlinked to ftcSDK TeamCode directory
- **Dependencies**: Add custom dependencies directly to `ftcSDK/TeamCode/build.gradle`

## 📦 **Adding Dependencies (e.g., Road Runner)**

Simply edit `ftcSDK/TeamCode/build.gradle` and add your dependencies:

```gradle
dependencies {
    implementation project(':FtcRobotController')
    
    // Add your custom dependencies here
    implementation 'com.acmerobotics:road-runner-core:0.9.6'
    implementation 'com.acmerobotics:road-runner-trajectory-sequence:0.9.6'
}
```

Then run `./build.sh` to build with your new dependencies.