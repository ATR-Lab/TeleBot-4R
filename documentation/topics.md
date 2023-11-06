# Topics
## Topic Naming Conventions
Topics should be named as follows:
```
system/level/subsystem/scope/topic/...
```
Each part of the topic name serves to build an understanding of where the topic originates from and how it can be used.
- **system**: which system the topic is coming from
- **level**: what level the subsystem is in, see [levels](systems.md#level) for more information
- **subsystem**: which subsystem the topic originates from
- **scope**: *public* or *private*. Publicly scoped topics are inputs or outputs to a subsystem. Other subsystems need to access these. Privately scoped topics are used internally in subsystems and should not be accessed by other subsystems.
- **topic**: a descriptive title of the topic in snake_case.