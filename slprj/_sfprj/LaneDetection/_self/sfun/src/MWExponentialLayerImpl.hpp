/* Copyright 2018 The MathWorks, Inc. */

#ifndef __EXPONENTIAL_LAYER_IMPL_HPP
#define __EXPONENTIAL_LAYER_IMPL_HPP

#include "MWCNNLayerImpl.hpp"

/**
  *  Codegen class for Exponential layer
**/
class MWCNNLayer;
class MWTargetNetworkImpl;
class MWExponentialLayerImpl : public MWCNNLayerImpl
{
  public:
    
    MWExponentialLayerImpl(MWCNNLayer*, MWTargetNetworkImpl*);
    ~MWExponentialLayerImpl();

    void predict();
    void cleanup();
    void propagateSize();
};



#endif
