---
title: python 模块、包及 import 方法
date: 2019-06-11 09:54:09
categories: Python Notes
tags: [Python]
---

## 模块和包

在 Python 中，一个 .py 文件就是一个模块（module）。使用模块大大提高了代码的可维护性，一个模块编写完成，就可以被其他项目代码引用。使用模块还可以避免函数名和变量名的冲突，相同名字的函数和变量可以分别存在于不同的模块中。我们在编写自己的模块时，不必考虑名字与其他模块内名字冲突。
<!--more-->
在创建许许多多模块后，我们可能希望将某些功能相近的文件组织在同一文件夹下，这里就需要运用包的概念了。简单的说，包就是一个文件夹，但该文件夹下必须有 \_\_init\_\_.py 文件（若无，则 Python 将其视为一个普通目录）。常见的包结构如下：  

```
 my_package/
    __init__.py
    a.py
    b.py
```

包可以有多级目录，组成多层次的包结构：

```
my_package/  
    __init__.py
    web/
        __init__.py
        www.py
        utils.py
    a.py
    utils.py
```

文件 `www.py` 的模块名就是 my_package.web.www，两个`utils.py` 的模块名分别是 my_package.utils 和 my_package.web.utils。

第一次导入包或者包的任何其他部分，会执行包中的 `__init__.py` 文件，这个文件可以为空，也可以存放一些初始化的代码。

`__init__.py` 可以为空，也可以执行包的初始化或者设置 `__all__` 变量的值。

## 导入模块

模块是最高级别的程序组织单元，它将程序代码和数据封装起来以便重用。模块可以由两个语句和一个重要的内置函数进行处理。  
- import: 使导入者以一个整体获取一个模块
- from: 允许导入者从一个模块文件中获取特定的变量名
- imp.reload: 在不中止 Python 程序的情况下，提供了一种重新载入模块文件代码的方法。

`import` 后面应该跟模块名，用法为`import module`或`from package import module`。在导入自建包的时候，如果直接写`import my_package`，`my_package.my_module.my_func()`，会报错`module 'my_package' has no attribute 'my_module'`，即 Python 将导入自建包的语句当作是导入一个模块。有一些操作可以让我们实现`import my_package`这种写法，在下面会做讨论。

首先，我们要先了解一下，当我们在 Python 代码中执行 import 的时候，Python 解释器做了什么。

有些 C 程序设计者喜欢把 Python 的模块导入操作比作 C 语言中的 `#include`，但其实不应该这么比较：在 Python 中，导入并非只是把一个文件文本插入另一个文件而已。导入其实是运行时的运算，程序第一次导入指定文件时，会执行三个步骤。
+ 找到模块文件。
+ 编译成位码。
+ 执行模块的代码来创建其所定义的对象。

这三个步骤旨在程序执行时，模块第一次导入时才会进行。在这之后，导入相同模块时，会跳过这三个步骤，而只提取内存中已加载的模块对象。更底层一些的说，Python 把载入的模块存储到一个名为 sys.modules 的表中，并在一次导入操作的开始检查该表，如果模块不存在，将会启动一个三步骤的过程：
1. 搜索  
   Python 使用标准模块搜索路径来找出 import 语句所对应的模块文件。在标准的 import 中引入路径从语法上讲是非法的\[[注](#zhushi)\]。模块搜索路径包括**程序主目录，PYTHONPATH 目录，标准链接库目录以及任何 .pth 文件的内容（按顺序依次搜索）**。这四者组合起来就是 sys.path。我们在有的地方看到会 import os，然后再 path 中添加模块所在的路径，就是在操作 sys.path。  
   <a name="zhushi">
   注：
   可以使用`from dir1.dir2 import module`或`import dir1.dir2.module`，这样加了路径的导入方式，但是前提是，dir1 位于上述四个搜索路径之中。
   </a>
2. 编译（可选）  
   找到源文件后，Python 会将其编译成字节码，若在搜索路径上只发现了字节码文件，而没有源代码，则会直接加载字节码（这意味着可以把一个程序模块作为字节码发布，使程序提速）。被导入的文件会产生一个 .pyc 字节码文件，被导入文件的字节码保存在文件中从而可以提高之后导入的速度。
3. 执行  
    模块文件中的所有语句会依次执行，任何对变量名的赋值运算，都会产生所得到的模块文件的属性。

### import 和 from

客户端可以执行 import 或 from 语句来使用模块文件，如果模块还没被加载，这两个语句就回去搜索、编译以及执行模块文件。主要的差别在于，import 会读取整个模块，from 将获取（或者说复制）模块中特定的变量名。

``` python
import module1
module1.printer('hello world!')

from module1 import printer
printer('hello world!')
```
这段代码展示了两种导入方式。import 语句使用一个变量名引用整个模块对象，所以必须通过模块名称（module1）来使用模块的属性（printer）。from 把模块中的变量名 printer 复制到另一个作用域，它让我们可以直接在脚本中使用复制后的变量名。**from语句有破坏命名空间的潜质。如果使用from导入变量，而那些变量碰巧和作用域中现有变量重名，变量就会被悄悄的覆盖掉。使用 import 语句时就不存在这种问题，因为必须通过模块名才能获取其内容。不过，使用from时，只要你了解并预料到可能发生这种事，在实际情况下这就不是一个大问题了，尤其是当你明确列出导入变量名时（例如，from module import x, y, z）。**

**import 和 from 是赋值语句！**  
**import 和 from 是赋值语句！**  
**import 和 from 是赋值语句！**  

就像 def 一样，import 和 from 是可执行的语句，而不是编译期间的声明，import 和 from 都是隐性的赋值语句。import 将整个模块对象赋值给一个变量名，例如 `import numpy as np`，变量 np 可以调用 numpy 里的对象。from 将一个或多个变量名赋值给另一个模块中的同名对象，例如`from random import random`，在当前模块中使用 random 变量就可以调用 random 模块的对象。

又因为 from 是复制引用模块内的变量，变成当前脚本中的对象的引用，所以通过 from 赋值的对象是不安全的：

``` python 
"""var.py"""
x = 1
y = [1,2]

"""test1.py"""
from var import x,y
...
x = 2
y[0] = 9
...

"""test2.py"""
import var
print(x)  # 1
print(y)  # [9,2]
```
列表 y 是一个可变对象，在 test1 中，改变了 y 的值，在 test2 中得到了反馈，test2 中的 y 被修改。

所以，通常会建议大家使用 import 而非 from 来导入模块（而不是导入包）。from 破坏了命名空间，造成了变量名的重复甚至覆盖，或者如上例所示，同名的变量无意间修改了 var 模块中的变量。这都是不安全的隐患。而 import 通过 module.x，module.y 这样的访问方式，大大提升了代码的安全性。


## 导入包

### \_\_init\_\_.py

要导入一个包，必须遵循一条约束：包的每一级目录都有 \_\_init\_\_.py 文件。

Python 首次导入某个包时，会自动执行该目录下 \_\_init\_\_.py 文件中的所有程序代码。因此，\_\_init\_\_.py 文件自然就应该是一份初始化代码。

举一个例子，我们看一下 Python Web 框架 django 中是怎么写 \_\_init\_\_.py 文件的：

``` python
"""django/django/apps/__init__.py"""
from .config import AppConfig
from .registry import apps

__all__ = ['AppConfig', 'apps']
```
包 apps 内有三个文件 `\_\_init\_\_.py`，`config.py`，`registry.py`。如果我们什么也不做，直接 import apps，是可以成功导入的，但是在使用的时候，`apps.config.AppConfig` 会报错，`module 'apps' has no attribute 'config'`，Python 在这里直接导入了一个模块，而未能按照包来处理。但是在 \_\_init\_\_.py 中加入那两行代码之后，`import apps`相当于执行了这两个 from 导入。另外一种写法就是，\_\_init\_\_.py 中不写 from 语句，在需要导入包的地方使用`from apps import config`，这样也可以实现模块的导入，同时也没有破坏命名空间。

再来看看 \_\_all\_\_ 变量。该变量定义了以 `from ... import *` 语句形式导入包时，需要导出什么。若定义了这个变量，那么使用`from ... import *`导入包时，并不会导入包内所有的模块的所有变量，而是只导出列表中的变量。


### 关于 import 的一些语法

+ 在同级目录导入模块时，不能使用 `from . import module`，应该直接写`import module`

+ 在同级目录下，可以写`from .module import attribute`

## Python 模块文件模板

这只是一个建议，摘自[廖雪峰Python教程](https://www.liaoxuefeng.com/wiki/1016959663602400/1017455068170048)。

``` python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'a test module '  # 任何模块代码的第一个字符串都被视为模块的文档注释

__author__ = 'LiBo'

import sys

def test():
    args = sys.argv
    if len(args)==1:
        print('Hello, world!')
    elif len(args)==2:
        print('Hello, %s!' % args[1])
    else:
        print('Too many arguments!')

if __name__=='__main__':
    test()
```

以上。
