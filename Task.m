classdef Task < handle

    properties
        xdotbar = [] % reference task velocity
        J = []       % task Jacobian
        A = []       % task internal activation function
        ap
        instant_activation
    end

    methods (Abstract)
        updateReference(obj)
        updateJacobian(obj)
        updateActivation(obj)
    end
end