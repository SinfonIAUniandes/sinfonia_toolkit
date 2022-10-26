# Mic Converter

## Main Dependencies:

- boost/foreach
- converter/converter_base
- message_actions
- naoqi_bridge_msgs/AudioBuffer


## registerCallback

Registers a MessageAction in the '_callbacks' map data structure, defined in the converter_base dependency.

### params:
1. MessageAction: action
2. CallbackT: Callback

### returns:
- void (no return).

## callAll

Calls all the MessageAction entities in the 'actions' vector, and registers along the 'message' AudioBufferPtr in the '_callbacks' map data structure.

### params:
1. vector<MessageAction> actions
2. AudioBufferPtr message

### returns:
- void (no return).