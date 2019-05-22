---
title: Python 随机数 seed
date: 2019-05-21 10:21:12
categories: Python Notes
tags: [Python]
---

若不设置 seed，则每次会生成不同的随机数；若设置了 seed，则相当于设置了随机数生成的起点，后续生成的随机数都将是确定的。   

<!--more-->

```python
import random

random.seed()
print ("使用默认种子生成随机数：{}".format(random.randint(1,5)))

random.seed(10)
for i in range(3):
    print ("使用种子10生成随机数：{}".format(random.randint(1,5)))
    print ("使用种子10生成随机数：{}".format(random.randint(1,5)))
print("******************************")
for i in range(3):
    random.seed(10)
    print ("使用种子10生成随机数：{}".format(random.randint(1,5)))
    print ("使用种子10生成随机数：{}".format(random.randint(1,5)))
```

第一个循环生成 6 个随机数：5，1，4，4，5，1。这相当于随机数种子 10 所能生成的随机数序列的前六个数字。第二个循环每次只取该序列的前两个数字：5，1。第二个循环的输出为：5，1，5，1，5，1。
