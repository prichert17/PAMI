#include "filter.hpp"

Filter_Vector2DAndRotation::Filter_Vector2DAndRotation():
    index(0){
    // Initialize the array of values to 0
    for(int i = 0; i<size; i++){
        values[i] = Vector2DAndRotation(0.0, 0.0, 0.0);
    }
}

void Filter_Vector2DAndRotation::add_elem(Vector2DAndRotation value){
    // Add an element to the filter and update the index
    values[index] = value;
    index = (index+1)%size;
}

Vector2DAndRotation Filter_Vector2DAndRotation::get_average(){
    // Compute the average of the values in the filter
    // TODO: Add a summed value to avoid recomputing the sum each time
    Vector2DAndRotation sum = Vector2DAndRotation(0.0, 0.0, 0.0);
    for(int i = 0; i<size; i++){
        sum += values[i];
    } 
    return sum/size;
}
