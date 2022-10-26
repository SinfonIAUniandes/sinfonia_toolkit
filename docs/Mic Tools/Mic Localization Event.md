# Mic Localization Event

## MicLocalizationEvent

initializes a MicLocalizationEvent. with name and session being initialized by 'params'

### params
1. string name
2. float& (reference) frecuency
3. SessionPtr& (reference) session

## startProcess

starts a concurrent process regarding the microphone. This includes a scoped lock being initiated.

## stopProcess

Terminates a concurrent process regardin the microphone. This includes liberating the concurrent lock.

## soundLocatedCallback

generates a callback, posting a message given the parameters of audio localization the robot percieves.