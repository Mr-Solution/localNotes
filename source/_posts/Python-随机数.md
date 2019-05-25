---
title: Python 随机数
date: 2019-05-21 10:21:12
categories: Python Notes
tags: [Python]
---

有这样一个场景：在蛇棋游戏中，总共有 100 个格子，其中有 ladder_num 个格子中有梯子，梯子的作用是让棋子直通到另外一个格子中。用 python 实现梯子功能的代码片段如下：
```python
ladders = dict(np.random.randint(1, 100, size=(ladder_num, 2)))
for k,v in ladders.items():
    ladders[v] = k
```
`np.random.randint(1,100,size=(ladder_num, 2))` 意思是随机生成一个 ladder_num x 2 的二维数组，数组中每一个元素的大小在 1 到 100 之间。然后将二维数组转换为 dict，其中 key 代表起始的格子编号，value 代表指向的格子编号，梯子由 key 指向 value。再用一个循环将梯子由 value 指向 key，说明梯子是双向的。  
这里有两个问题。第一，随机数是不可控的，在 ladder_num 个数据对中很有可能会出现相同的数字，`dict()` 函数是有去重功能的，假设存在数据对 `[[2,8],[2,15]]`，那么最终保留在 dict 中的 key 值 2，对应的 value 只有 15。这就造成了数据丢失，若我们的目的是为了生成 10 个梯子，这样最终只能生成 9 个梯子。第二，假设随机生成的数据对为`[...,[45,48],[65,68],[68,10],...]`，那么后面的循环就会出现问题，因为`dict`中 key 65 对应的 value 为 68，循环中反转 kv，得到一个 key 68 指向 value 65，但是`dict`中已经有值为 68 的 key 了，它指向 value 10，这里就对原来的梯子做了修改。更严重的是，这里会报错`RuntimeError: dictionary changed size during iteration`，这是因为 Python 3 是不允许在遍历中修改字典大小的。  

可以将代码修改如下：
```python
ladders = np.random.randint(1, 100, size=(ladder_num, 2))
for k,v in ladders:
    ladders = np.vstack((ladders, np.array([v,k])))
ladders = dict(ladders)
```
但是这样依然无法解决第一个问题，ladders 这个字典的大小依然可能小于我们希望的 ladder_num。为了得到不重复的 N 个随机数，我们有两个方法：第一，使用`random.sample(range(A,B),N)`函数，表示从[A,B]区间随机生成 N 个数，结果以列表返回。第二，使用 numpy 的函数`np.random.sample(size)`，该函数返回一个 size 数组，元素在[0.0, 1.0)区间：
```python
ladders = np.random.sample((ladder_num, 2)) * (100 - 0)
ladders = ladders.astype(int)
for k,v in ladders:
    ladders = np.vstack((ladders, np.array([v,k])))
ladders = dict(ladders)
```


### 关于随机数 seed

若不设置 seed，则每次会生成不同的随机数；若设置了 seed，则相当于设置了随机数生成的起点，后续生成的随机数都将是确定的。   

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
