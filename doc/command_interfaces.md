
**** This is obsolete but kept for later improvement ***
***************************************************


# Command interface

## Commands

Issue:

Commands need to be

- checked for legitimacy

- serialized to one or many CAN messages

- registered as pending and / or substates

- responses need to be registered into state



Idea:

- Commands should implement an interface

- checking and serialization should be done by standard methods

- some commands are naturally exclusive, others perhaps not - exclusivity flag ?

- however I do not like malloc pressure at this level

- how to do creation and memory management - is one
free list / pool for each class good? Can different commands
be lumped together into a union ?


- Can we / need we / should we associate result states
with certain commands?

## Responses

- responses could use a lookup table with registered methods

-- deserialization
-- state transitions: switch or another table ?

- Is a table of method pointers The Right Thing here?

Alas: responses can lead to re-sending of commands!
(time-outs, datum search branches, minor collisions)

--> how can the cycling be bounded ?

- some commands complete when individual FPUs respond,
some complete when all functional ones complete






