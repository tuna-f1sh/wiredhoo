#ifndef POLYFIT_H_
#define POLYFIT_H_

int polyfit(const float* const dependentValues,
            const float* const independentValues,
            unsigned int        countOfElements,
            unsigned int        order,
            float*             coefficients);

#endif
