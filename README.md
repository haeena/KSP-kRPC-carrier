# 
```
ln -s Ships "${PATH_TO_KSP_SAVE}\Ships"
```

```
New-Item -ItemType Junction -Value Ships -Path "${PATH_TO_KSP_SAVE}\${SAVE_NAME}\Ships"
```