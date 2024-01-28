
def attach_debugger() -> bool:
    try:
        import debugpy
        # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
        debugpy.listen(('0.0.0.0', 5678))
        print("Waiting for debugger attach")
        debugpy.wait_for_client() 
        print('Debugger attached')
    except ModuleNotFoundError:
        print("debugpy not available.  Add debugpy to pyproject.toml required packages.  Then sync robot with 'python -m robotpy sync' to install on RoboRIO.")
        return False
    except Exception as e:
        print(f"Failed to attach debugger. Exception:\n{e}")
        return False
    
    return True