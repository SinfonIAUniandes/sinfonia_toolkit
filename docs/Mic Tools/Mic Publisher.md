# Mic Publisher

Publisher for the Microphone. mic_localization_publisher manages a comparable structure.

## Main Elements:

- _isInitialized: bool, wether the mic is initialized or not.
- _topic: String, the topic where MicPublisher is initialized.

## isInitialized

returns a bool object, corresponding to the value of '_isInitialized'

## topic

returns the '_topic' value of object.

## isSubscribed

returns wether something is subsctibed to MicPublisher given the '_publisher' value being higher than 0. If MicPublisher is not initialized, this value will Always return False.

## publish

publishes a message (via accessing its pointer) through '_publisher'

### params:
1. AudioBufferPtr message

### returns:
- void (no return)

## reset

resets the MicPublisher through 'NodeHandle'.

### params:
1. NodeHandle (pointer): nodeHandle

### returns:
- void (no return)

## shutdown

shuts down MicPublisher.

