---
title: Effective C++ 读书笔记 03：尽可能使用const
date: 2019-01-22 15:48:49
categories: 编程语言
tags: [C++,读书笔记]
---

>const 允许你指定一个语义约束，而编译器会强制执行这项约束，确保某个对象不会被改变。如果某个对象在执行中不应该（或者不会）被改变，就应该使用 const。

<!--more-->

## 一. const 修饰变量

看下面一段代码：

```C++
#include<iostream>
using namespace std;
int main() {
    int a = 1; //non-const data
    const int b = a; //const data
    int *c = &a; //non-const data,non-const pointer
    const int *d = &a; //const data,non-const pointer
    int * const e = &a; //non-const data,const pointer
    const int * const f = &a; //const data,const pointer
    const int * const g = &a; //const data,const pointer
    return 0;
}
```

``const`` 修饰指针的时候，``const`` 在 \* 号右边表示指针是 ``const`` 的，在 \* 左边，表示 \*p（即指针指向的对象），是 ``const`` 的。在 STL 迭代器的应用中，``map<int, int>::const_iterator itr`` 相当于 ``const T* itr``，即迭代器（指针）``itr`` 指向的对象 ``*itr`` 是 ``const`` 的，而 ``const map<int, int>::iterator itr`` 相当于 ``T* const itr``，即迭代器 ``itr`` 是 ``const`` 的。

在上边代码中，若 ``a`` 的值被改变了，那么除 ``b`` 外，``*c``， ``*d``， ``*e``， ``*f``， ``*g`` 都会改变，因为 ``b`` 的赋值是 copy 赋值立即数，之后的 ``b`` 就跟 ``a`` 没有关系了，但对 ``c-g`` 而言，修饰它们的 ``const`` 仅仅表示，不能修改指针的值或者通过 ``*p`` 修改指针指向的变量，但 ``a`` 并不是 const data，所以可以通过修改 ``a`` 的值来改变指针 ``cdefg`` 指向的值。

使用 ``const`` 需要注意，``const`` 对象必须初始化（因为它不能改变不能接受再次的赋值）（但并不一定在声明的同时给予初值）。在函数中声明  ``const`` 需要立即给初值，在类 class 中声明的时候，不能立即赋值，这是因为 ``const`` 数据成员只在某个对象生存期内是常量，而对于整个类而言却是可变的（这点与 ``static`` 相反），类可以创建多个对象，相应的 ``const`` 成员的初始化只能在类的构造函数的初始化表中进行：

```C++
class Stuff {
public:
    Stuff(int m) : max_num(m)
        { };
private:
    const int max_num;
};
```

要想建立在整个类中都恒定的常量，应该用类中的枚举常量或者 ``static`` 来实现。

## 二. const 修饰成员函数

``const`` 成员函数的声明：``const`` 关键字只能放在函数声明的尾部！

```c++
int func() const {
    return 0;
}
```

 ``const`` 修饰的成员函数不能修改这个类对象的任何的数据成员（准确地说是非静态数据成员）。

 ``const`` 成员函数不能调用非 ``const`` 成员函数，因为非 ``const`` 成员函数可以会修改成员变量。

  在设计类的时候，一个原则就是对于不改变数据成员的成员函数都要声明为``const`` 成员函数。有 ``const`` 修饰的成员函数（``const`` 放在函数参数表的后面，而不是在函数前面或者参数表内），只能读取数据成员，不能改变数据成员；没有 ``const`` 修饰的成员函数，对数据成员则是可读可写的。

这样做好处在于，首先，类的接口更容易被理解，一看便知函数是读操作还是有写操作；其次，将成员函数声明为 ``const`` 使得该函数操作 ``const`` 对象成为可能，因为常量（即 ``const``）对象可以调用 ``const`` 成员函数，而不能调用非 ``const`` 修饰的函数。

ps：之所以我们希望函数可以操作 ``const`` 对象是因为处于程序的效率考虑，传引用或者指针往往比传值要好，传引用的时候为了防止对象被修改，往往要声明为 ``const`` 引用或指针，``const`` 对象不能调用非 ``const`` 函数，所以，尽可能的将只有读操作的函数声明为 ``const`` 函数。

注意：两个成员函数如果只是常量性不同，是可以被重载的，原因在于函数的形参列表里隐藏有 ``this`` 指针，``const`` 函数里 ``this`` 指针是指向 ``const`` 对象的指针，而非 ``const`` 函数里的 ``this`` 指针是正常版本的指针。

```C++
#include<iostream>
using namespace std;
class Chinese {
public:
    Chinese() {
        personID = IdentityCardId;
        IdentityCardId++;
    }
    int getNation() const{
        cout << "CHINA ID : " << personID << endl;
        IdentityCardId++;
        return 0;
    }
    int getNation() {
        cout << "china ID : " << personID << endl;
        return 0;
    }

private:
    static int IdentityCardId;
    int personID;
};

int Chinese::IdentityCardId = 1;

int main() {
    const Chinese LaoWang = Chinese();
    LaoWang.getNation(); //输出CHINA ID:1，const对象LaoWang默认调用const成员函数，并且可以修改static变量IdentityCardId
    Chinese LaoSun = Chinese();
    LaoSun.getNation(); //输出china ID:3，non_const对象LaoSun默认调用non_const的成员函数
    return 0;
}
```

``const`` 对象默认调用 ``const`` 成员函数，非 ``const`` 默认调用 ``const`` 成员函数，类中只有一个函数存在的情况下（我们删掉 non_const 版本的 ``getNation()``），non_const 的对象也可以调用非 const 成员函数。

## 三. const 修饰函数返回值

如果给以“指针传递”方式的函数返回值加 ``const`` 修饰，那么函数返回值（即指针）的内容不能被修改，该返回值只能被赋给加 ``const`` 修饰的同类型指针。例如函数:

```C++
const char * GetString(void);
```

如下语句将出现编译错误：

```C++
char *str = GetString();
```

正确的用法是:

```C++
const char *str = GetString();
```

如果函数返回值采用“值传递方式”，由于函数会把返回值复制到外部临时的存储单元中，加 ``const`` 修饰没有任何意义。

例如不要把函数:

```C++
int GetCnt();
```

写成:

```C++
const int GetCnt();
```

同理不要把函数(T为自定义数据类型):

```C++
T getT(void);
```

写成

```C++
const T getT(void);
```

如果返回值不是内部数据类型，将函数 ``T getT(void)`` 改写为 ``const T & getT(void)`` 的确能提高效率。但此时千万千万要小心，一定要搞清楚函数究竟是想返回一个对象的“拷贝”还是仅返回“别名”就可以了，否则程序会出错。

函数返回值采用“引用传递”的场合并不多，这种方式一般只出现在类的赋值函数中，目的是为了实现链式表达。（除了重载操作符外一般不要将返回值类型定为对某个对象的 ``const`` 引用！）

例如：

```C++
class A {
    A & operate = (const A &other); // 赋值函数
};

A a, b, c; // a, b, c 为A 的对象
a = b = c; // 正常的链式赋值
(a = b) = c; // 不正常的链式赋值，但合法
```

如果将赋值函数的返回值加 ``const`` 修饰，那么该返回值的内容不允许被改动。上例中，语句 ``a = b = c`` 仍然正确，但是语句 ``(a = b) = c`` 则是非法的。
