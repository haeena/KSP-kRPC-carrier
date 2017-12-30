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
# on Windows
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt

# on OS X
python -m venv venv
source vev/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## Manage your savedata under git
```
ln -s saves/${SAVE_NAME} "${PATH_TO_KSP_SAVE}\${SAVE_NAME}"
```

```
New-Item -ItemType Junction -Path saves/${SAVE_NAME} -Value "${PATH_TO_KSP_SAVE}/${SAVE_NAME}"
```