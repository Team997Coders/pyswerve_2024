

Create a virtual environment for the project.  Use the command palette CTRL+SHIFT+P and type "Python: Create Virtual Environment"

Enable unrestricted execution policy for the current user
```Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser

--- Install robotpy on the development system ---
Open a terminal in VS Code
```pip install robotpy```

--- Install python on the robot ---
```py -3 -m robotpy sync```


