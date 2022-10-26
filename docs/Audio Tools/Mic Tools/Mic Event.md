# Mic Event

## MicEventRegister

Registers a new event that requires or otherwise uses mic functions, setting most important values to their default values.

### params:

1. string& (reference): name
2. float& (reference): frecuency
3. SessionPtr& (reference): session

## ~MicEventRegister (destroyer)

stops a MicEventRegister entity, terminating its execution.

## resetPublisher

resets the publisher associated to the Mic Event.

## shutdownPublisher

shuts down the publisher associated to the Mic Event.

## timerCallback

prints the '_counter' variable, then sets it back to 0.

## initSpeechRecognition

initializes the Speech Recognition service, this service is started in a concurrent way, able to run in a separate Thread.

## stopSpeechRecognition

terminates the Speech Recognition service, also terminating the concurrent lock the service has in a Thread.

## startProcess

starts a concurrent process regarding the microphone. This includes a scoped lock being initiated.

## stopProcess

Terminates a concurrent process regardin the microphone. This includes liberating the concurrent lock.

## isPublishing

Checks the 

## isStarted

Returns wether this MicEventRegister instance has been started.

## setDefaultParameters

sets the default parameters for '_micSampleRate' and '_channels'.

## setParameters

sets parameters for '_micSampleRate' and '_channels' to the parameters sent through a vector.

## processRemote

Sends processes to remote listeners.

## wordRecognizedCallback

reacts to the recognition of a word, showing the value and confidence on such.

### params
1. string: key
2. AnyValue: value
3. string: subscriberIdentifier

## soundDetectionCallback

reacts to the recognition of a sound, giving a callback.

### params
1. string: key
2. AnyValue: value
3. string: subscriberIdentifier