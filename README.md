# 项目简要

## 碎碎念
该仓库代码由上海交通大学自动化系学生维护:yum:


## 前情提要
因为老师给了控制小车的MecanumDriver类，所以我们控制舵机需要使用该类的方法。然而该类中
`analogWrite(pin1, constrain(abs(speed), 0, 100));`等相关代码使得小车舵机速度有上限，实则可以更快，于是我把该类的相关代码进行了修改，使得小车舵机可以更快地运行。
```C++
analogWrite(pin1, constrain(abs(speed), 0, 1000));
//使得小车舵机速度上限为1000，远超原先100的上限
```


## 项目文件结构
```bash
OurCodes/
├── README.md                         
├── LICENSE.txt                       
├── .gitattributes                   
├── .gitignore                       
├── file_structure.txt               
│
├── 八向避障/                          
│   └── exp1/
│       ├── exp1.ino                  
│       ├── MecanumDriver.h          
│       └── MecanumDriver.cpp         
│
├── 回字型/                           
│   └── exp2/
│       ├── exp2.ino                  
│       ├── MecanumDriver.h           
│       └── MecanumDriver.cpp         
│
├── 八字形/                           
│   └── exp3/
│       ├── exp3.ino                  
│       ├── MecanumDriver.h           
│       └── MecanumDriver.cpp         
│
├── out/                              
│   └── build/
│       └── x64-debug/               
│
└── .vs/                             
    └── ...
```