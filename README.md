# Setup instruction

## prerequiste
* KSP 1.3.1
* Python 3.6+

## install required mods via [ckan](https://github.com/KSP-CKAN/CKAN/wiki/User-guide)
```
ckan.exe install -c requirements.ckan
```

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