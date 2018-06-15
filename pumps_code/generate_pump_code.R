### libraries
require(tidyverse)
require(magrittr)

### work through permutations
perm = function(c1, c2) {
	ret = crossing(P1=c(F,T), P2=c(F,T), P3=c(F,T), P4=c(F,T))
	
	if(c1==0) ret %<>% filter(!P1 & !P2)
	if(c1==1) ret %<>% filter(xor(P1,P2)) 
	if(c1==2) ret %<>% filter(P1 & P2)
	
	if(c2==0) ret %<>% filter(!P3 & !P4)
	if(c2==1) ret %<>% filter(xor(P3,P4)) 
	if(c2==2) ret %<>% filter(P3 & P4)
	
	ret %<>% mutate(
		Case=1:n()-1,
		P1=factor(P1, levels=c(F,T), labels=c("OFF","ON")),
		P2=factor(P2, levels=c(F,T), labels=c("OFF","ON")),
		P3=factor(P3, levels=c(F,T), labels=c("OFF","ON")),
		P4=factor(P4, levels=c(F,T), labels=c("OFF","ON"))
	)
	
	return(ret)
}

cases = tibble(
		Lower = c(0,0,1,1,2),
		Upper = c(0,1,1,2,2)
	) %>%
	arrange(Lower+Upper, Lower) %>%
	mutate(
		Level = 1:n()-1,
		pumps = map2(Lower, Upper, perm)
	) %>%
	unnest(pumps) %>%
	mutate( Index = 1:n()-1 ) %>%
	select( Index, Level, Case, Lower, Upper, everything() ) %T>% print

wline = function(...) { cat(paste0(..., collapse="")); cat('\n') }

for( lvl in unique(cases$Level) ) {
	s = cases %>% filter(Level == lvl)
	
	wline("void setLevel_", lvl, "(pumpState &ps[N_PUMP]) {")
	
	wline("const byte maxCase = ", max(s$Case), ";")
	wline("static byte case = 0;\n")
	wline("const pumpState s[][N_PUMP] = {")
	for( case in s$Case ) {
		f = s %>% filter(Case==case)
		wline("  {",f$P1,",",f$P2,",",f$P3,",",f$P4,"}",ifelse(case<max(s$Case),",",""))
	}
	wline("};\n")
	
	wline("ps = s[case++];")
	wline("if( case>maxCase ) case=0;")
	
	wline("}",'\n')
}

