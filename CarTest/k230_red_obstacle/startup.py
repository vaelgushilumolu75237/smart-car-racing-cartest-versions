# Start the K230 red obstacle camera from the installed package.
import os
import sys
root = '/sdcard/cartest-k230'
if root not in sys.path:
    sys.path.insert(0, root)
os.chdir(root)
try:
    namespace = {'__name__': 'cartest_red_camera', '__file__': root + '/main.py'}
    with open(root + '/main.py', 'r') as stream:
        source = stream.read()
    exec(source, namespace)
    namespace['run']()
except Exception as error:
    sys.print_exception(error)
    with open(root + '/startup_error.txt', 'w') as stream:
        sys.print_exception(error, stream)
    raise
