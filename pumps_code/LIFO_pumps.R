### libraries
require(tidyverse)
require(magrittr)

lowerState = c(F,F)
lowerNext = rbinom(1, 1, 0.5)
lowerAdd = function() {
	if( sum(lowerState) >= 2 ) return
	lowerState[lowerNext+1] <<- T
	lowerNext <<- !lowerNext
}
lowerSub = function() {
	if( sum(lowerState) <= 0 ) return
	if( sum(lowerState) == 1 ) lowerNext <<- !lowerNext
	lowerState[lowerNext+1] <<- F
	lowerNext <<- !lowerNext
}
lowerSub(); lowerSub();
print(lowerState)
lowerAdd(); print(lowerState)
lowerSub(); print(lowerState)
lowerAdd(); print(lowerState)
lowerSub(); print(lowerState)

lowerSub(); lowerSub();
print(lowerState)
lowerAdd(); print(lowerState)
lowerAdd(); print(lowerState)
lowerSub(); print(lowerState)

lowerAdd()
print(lowerState)
lowerAdd()
print(lowerState)
