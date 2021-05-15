#!/usr/bin/python
def strList2int(strList):
    if isinstance(strList, list):
        return list(map(strList2int, strList))
    else:
        return int(strList)
