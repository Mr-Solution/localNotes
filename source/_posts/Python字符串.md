---
title: Python字符串
date: 2019-05-21 10:05:51
categories: Python Notes
tags: [Python]
---

字符串的访问跟列表一样，用方括号访问，支持切片操作。  
python 支持格式化字符串的输出，最基本的用法是字符串中插入 `%s`：
```python
print("My name is %s." % ("LiLei"))
str = "Her name is %s" % ("HanMeimei")
```
具体的字符串格式符同 C 语言一样。  
从 Python2.6 开始，新增了一种格式化字符串的函数 `str.format()`，它增强了字符串格式化的功能。其基本语法是通过 `{}` 和 `:` 来代替以前的 `%`。`format()` 函数可以接受不限个参数，位置可以不按顺序。  
```python
>>>"{} {}".format("hello", "world")  # 不设置指定位置，按默认顺序
'hello world'
>>>"{0} {1}".format("hello", "world")  # 设置指定位置
>>>"{0} {1} {0}".format("hello", "world")
hello world hello
```
