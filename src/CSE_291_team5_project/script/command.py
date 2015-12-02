import constant as C


def contains(txt,strings):
    for s in strings:
        if s in txt:
            return True
    return False


def parse(text):
    for cmd in C.Commands:
        if contains(text,cmd[1]):
            return cmd[0]
    return None
            
