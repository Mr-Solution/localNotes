---
title: python print一个进度条
date: 2019-07-09 19:51:01
categories: Python Notes
tags: Python
---

我们都知道`print('\n')`表示换行，转义字符`\n`对应的意义就是换行，将当前输出位置移到下一行的开头。
<!-- more -->

为了实现进度条的功能，我们要在`print()`函数中使用另一个转义符`\r`。`\r`的意义是回车，将当前输出位置移到**当前**行的开头。所以我们可以使用`print(\r)`来使打印内容在同一行循环出现。

但是，我们应该注意到，在使用`print()`函数的时候，其实不需要参数`\n`也会输出换行。这是因为，`print()`函数有一个默认参数`end='\n'`，如果不显式指明`end`的值，那么默认输出换行。所以我们要想实现代码在一行循环出现，应该这样写：
``` Python
for i in range(10):
    print(u"当前进度为 {}/10".format(i), end='\r')  # u表示字符串为unicode字符串，输出中文时防止乱码
    time.sleep(0.25）
```
了解了其中的原理，再写一个进度条就很容易了：
``` Python
import time
count_down = 10  # 倒计时时间，单位：秒
interval = 1     # 屏幕刷新的间隔时间，单位：秒
for i in range(0, int(count_down/interval)+1):
    print("#"*i + " " + "%.2f"%(i/(count_down/interval)*100) + "%", end='\r')
    # 因为浮点数的除法会出现结果不准确的情况，故使用 %.2f 规则化输出
    time.sleep(interval)
print("\n加载完毕")
```

同进度条类似，我们可以实现一个在命令行“转圈”的等待效果：
``` Python
import time
count_down = 10  # 设置倒计时时间，单位：秒
interval = 0.25  # 设置屏幕刷新的间隔时间，单位：秒

for i in range(0, int(count_down/interval)): 
    ch_list = ["\\", "|", "/", "-"]
    index = i % 4
    print("\rinstalling " + ch_list[index], end="")
    time.sleep(interval)

print("\rinstallation accomplished.")
```

以上。