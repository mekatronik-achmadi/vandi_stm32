# Program Deployment Guide

## Table of Contents

- [Getting Sources](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/DEPLOYMENT.md#getting-sources).
- [Updating Sources](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/DEPLOYMENT.md#updating-sources).
- [Source Structure](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/DEPLOYMENT.md#source-structure).

-------------------------------------------------------------------

### Getting Sources

To get sources, you should use Git Clone method over just zip download.

First, right click on empty space in Windows Explorer -> _Git GUI Here_ -> _Clone Existing Repository_

Then fill _Source Location_ with this repo URL and
for _Target Directory_ you can choose wherever you like as long that directory still not created.
Lastly, click _Clone_.

![images](images/gitclone0.png?raw=true)

Wait for a while, then it will finish cloning from internet

![images](images/gitclone1.png?raw=true)

-------------------------------------------------------------------

### Updating Sources

Before able to do source pull, first tell the git who you are.
From _Edit_ -> _Options_, fill Username and Email on _Global (All Repository)_.
Click _Save_ at the end.

![images](images/gitpull0.png?raw=true)

Updating sources from Github using Git GUI consist two steps:
- Fetch using _Remote_ -> _Fetch from_ -> _origin_
- Wait for a while.
- then Merge using _Merge_ -> _Local Merge_

If no any commit conflict, the sources should be updated

![images](images/gitpull1.png?raw=true)

-------------------------------------------------------------------

### Source Structure.

The main directory when the sources resides is _rework_ directory.

Before you can compile the sources in there, you need to put ChibiOS/RT library first by extract the downloaded package there.
The sources tree should look like this:

~~~
STM32
+-- cringe-work
+-- guides
+-- rework
|   +-- ChibiOS_STM32
|   |   +-- demos
|   |   +-- ext
|   |   +-- os
|   |   +-- ...
|   +-- qtcreator
|   |   +-- rework.creator
|   |   +-- rework.files
|   |   +-- rework.includes
|   |   +-- ...
|   +-- chconf.h
|   +-- halconf.h
|   +-- main.c
|   +-- Makefile
|   +-- mcuconf.h
|   +-- ...
+-- LICENSE
+-- ...
~~~

-------------------------------------------------------------------

### Project Opening.

#### QtCreator

The QtCreator project file is a *.creator.
In this case, *rework/qtcreator/rework.creator* is the file.
Just open it from _File_ -> _Open File or Project_ then navigate it to file *.creator.

![images](images/prjqt.png?raw=true)

#### Programmer Notepad

Programmer Notepad usually doesn't use project style.
Just open the *Makefile*

![images](images/prjqt.png?raw=true)
