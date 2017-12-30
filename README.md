# Setup instruction

## prerequiste
* KSP 1.3.1
* Python 3.6+

## install required mods via [ckan](https://github.com/KSP-CKAN/CKAN/wiki/User-guide)
```
ckan.exe install -c requirements.ckan
```

[some additional resource required for InfernalRobotics](https://forum.kerbalspaceprogram.com/index.php?/topic/104535-112-magic-smoke-industries-infernal-robotics-202/&page=93&tab=comments#comment-3140574)
[This save](saves/default) uses `IR Rework Parts` and `Surface Sampler dll`

## setup kPRC venv and install requirements
```
# on Windows (PowerShell)
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt

# on macOS
python -m venv venv
source vev/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## Manage your savedata under git
```
# on Windows (PowerShell)
New-Item -ItemType Junction -Value saves/${SAVE_NAME} -Path "${PATH_TO_KSP_SAVE}/${SAVE_NAME}"

# on macOS
ln -s $(pwd)/saves/${SAVE_NAME} "${PATH_TO_KSP_SAVE}\${SAVE_NAME}"
```