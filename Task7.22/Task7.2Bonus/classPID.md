
# Class PID


[**Class List**](annotated.md) **>** [**PID**](classPID.md)



_class for implementing_ [_**PID**_](classPID.md) _controller_

* `#include <PID.h>`















## Public Functions

| Type | Name |
| ---: | :--- |
|  void | [**antiWindup**](classPID.md#function-antiwindup) () <br>_Limit accumulation of integral term._  |
|  void | [**computeOutput**](classPID.md#function-computeoutput) (double feedBack) <br>_Compute the_ [_**PID**_](classPID.md) _control loop._ |
|  void | [**displayOutput**](classPID.md#function-displayoutput) () <br>_Display the outout._  |
|  void | [**setParameters**](classPID.md#function-setparameters) (double Kp, double Ki, double Kd, double setPoint, double myLimits) <br>_Set the_ [_**PID**_](classPID.md) _parameters and setpoint._ |








## Public Functions Documentation


### <a href="#function-antiwindup" id="function-antiwindup">function antiWindup </a>


```cpp
void PID::antiWindup () 
```



### <a href="#function-computeoutput" id="function-computeoutput">function computeOutput </a>


```cpp
void PID::computeOutput (
    double feedBack
) 
```




**Parameters:**


* `feedBack` The feedback value from the analog sensor. 




        

### <a href="#function-displayoutput" id="function-displayoutput">function displayOutput </a>


```cpp
void PID::displayOutput () 
```



### <a href="#function-setparameters" id="function-setparameters">function setParameters </a>


```cpp
void PID::setParameters (
    double Kp,
    double Ki,
    double Kd,
    double setPoint,
    double myLimits
) 
```




**Parameters:**


* `Kp` The proportional term. 
* `Ki` The integral term. 
* `Kd` The derivative term. 
* `setPoint` The desired setpoint. 
* `myLimits` The limit for integral term to avoid integral windup 




        

------------------------------
The documentation for this class was generated from the following file `PID.h`