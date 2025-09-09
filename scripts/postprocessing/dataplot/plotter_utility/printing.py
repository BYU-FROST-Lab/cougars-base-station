def update(message, keepMessage=False):
    if (keepMessage):
        print(message + (' ' * 30))
    else:
        print(' ' + message + (' ' * 30), end="\r")