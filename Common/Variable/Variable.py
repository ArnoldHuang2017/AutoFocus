# -- coding: utf-8 -
class VariableContainer(dict):
    '''重写字典，既可以用访问成员变量的形式，也可以用下标的形式'''
    def __getattr__(self, item):
        return self.get(item, None)

    def __setattr__(self, key, value):
        self[key] = value


def AddContainer():
    '''
    创建新的容器
    '''
    return VariableContainer()


def Dict2Container(srccontainer, desdict):
    '''
    将字典转换为容器
    '''
    for k, v in desdict.items():
        if isinstance(v, dict):
            container = AddContainer()
            Dict2Container(container, v)
            srccontainer[k] = container
        else:
            srccontainer[k] = v


VAR = AddContainer()
VAR.Param = AddContainer()
VAR.Lens = AddContainer()

