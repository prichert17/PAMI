#ifndef __FILTER__
#define __FILTER__

#include "constants.hpp"
#include "coordinates.hpp"

/*
    This class is used to filter a generic type with a rolling average filter
*/
template <class T>
class Filter_Generic{
    private:
        const uint8_t size = CONSTANTS::FILTER_SIZE;    // Size of the filter
        T values[CONSTANTS::FILTER_SIZE];   // Array of values to filter   
        uint8_t index;  // Index of the last value added

    public:
        /**
         * @brief Construct a new Filter_Generic object
         *  */
        Filter_Generic():
        index(0){
            // Initialize the array of values to 0
            for(int i = 0; i<size; i++){
                values[i]=0.0;
            }
        }

        /**
         * @brief Add an element to the filter
         * @param value Value to add to the filter
         */
        void add_elem(T value){
            // Add an element to the filter and update the index
            values[index] = value; 
            index = (index+1)%size;
        }

        /**
         * @brief Get the average of the values in the filter
         * @return Average of the values in the filter
         */
        T get_average(){
            // Compute the average of the values in the filter
            // TODO: Add a summed value to avoid recomputing the sum each time
            T sum = 0.0;
            for(int i = 0; i<size; i++){
                sum += values[i];
            } 
            return sum/size;
        }
};

/*
    This class is used to filter a Vector2DAndROtation with a rolling average filter
*/
class Filter_Vector2DAndRotation{
    private:
        const uint8_t size = CONSTANTS::FILTER_SIZE;    // Size of the filter
        Vector2DAndRotation values[CONSTANTS::FILTER_SIZE]; // Array of values to filter
        uint8_t index;  // Index of the last value added

    public:
        /**
         * @brief Construct a new Filter_Generic object
         *  */
        Filter_Vector2DAndRotation();

        /**
         * @brief Add an element to the filter
         * @param value Value to add to the filter
         */
        void add_elem(Vector2DAndRotation value);

        /**
         * @brief Get the average of the values in the filter
         * @return Average of the values in the filter
         */
        Vector2DAndRotation get_average();
};

#endif