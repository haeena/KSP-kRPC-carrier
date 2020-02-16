# Setup instruction

## prerequiste

* KSP 1.9.0
* Python 3.8+

## install mods

### via [ckan](https://github.com/KSP-CKAN/CKAN/wiki/User-guide)

```
ckan.exe install ModuleManager
```

### install unofficial kRPC build for KSP 1.9

get it from https://github.com/haeena/krpc/releases/tag/v0.4.9.1

1. extract krpc-0.4.9.1.zip into to GameData folder on your KSP install directory
2. copy krpc-python-0.4.9.1.zip next to this README.md file


## setup kPRC venv and install requirements

### on Windows (PowerShell)
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt
pip install krpc-python-0.4.9.1.zip
```

### on macOS

```
python -m venv .venv
source .vev/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install krpc-python-0.4.9.1.zip
```

## Manage your savedata under git
```
# on Windows (PowerShell)
New-Item -ItemType Junction -Value saves/${SAVE_NAME} -Path "${PATH_TO_KSP_SAVE}/${SAVE_NAME}"

# on macOS
ln -s $(pwd)/saves/${SAVE_NAME} "${PATH_TO_KSP_SAVE}\${SAVE_NAME}"
```