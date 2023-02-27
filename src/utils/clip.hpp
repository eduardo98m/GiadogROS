#include <cmath>
#include <stdexcept>
/** 
* @brief: Clips a value `x` between two values `upper_bound` and `lower_bound`, 
*         all parameters must be the same type.
* @param: x           -> int/double/float Value to be clipped
* @param: upper_bound -> int/double/float Upper bound
* @param: lower_bound -> int/double/float Lower bound
* 
**/
template<class T>
T clip(T x, T upper_bound, T lower_bound){
    
    if (upper_bound < lower_bound){
        throw std::invalid_argument(
            "Error on clip function call: \n "
            "upper_bound smaller than lower_bound");
    };

    return std::max(lower_bound, std::min(x, upper_bound));  
}