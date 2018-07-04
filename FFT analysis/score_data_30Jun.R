### libraries
require(tidyverse) 
require(magrittr)
require(ggthemes)
require(skimr)
require(ROCR)

### numbers
nArch = 3
nFreq = 16
nSens = 6

### load data
d.o = read_csv('data_30Jun18.csv',
	col_names = c(
		"Notes","Time","Arch",
		sprintf("power_%s",(1:nSens)-1)
	)) %>%
#	filter( ! Notes %in% c("test","NOP","idle") ) %>%
	arrange(Time) %T>% print

foo = d.o %>% filter(Notes == "c01") %>% slice(1) 
	
foo %>% select(starts_with("power")) %>% as.matrix %>% t %>% data_framecor

d=d.o
smooth=10
reads = matrix(0, ncol=nArch, nrow=nSens)
d$P0=NA
d$P1=NA
d$P2=NA
d$C01=NA
d$C12=NA
d$C20=NA
for( i in 1:nrow(d) ) {
#	browser()

	read = d %>% slice(i) %>% select(starts_with("power")) %>% as.matrix %>% t

	ind = d$Arch[i]+1
	reads[,ind] = (reads[,ind]*(smooth-1)+read)/smooth

	d$P0[i]=mean(reads[,1])
	d$P1[i]=mean(reads[,2])
	d$P2[i]=mean(reads[,3])
	d$C01[i]=cor(reads[,1],reads[,2])
	d$C12[i]=cor(reads[,2],reads[,3])
	d$C20[i]=cor(reads[,3],reads[,1])
}
d

res = d %>% 
	select(Notes, Time, P0, P1, P2, C01, C12, C20) %T>% print

res %>%
	gather( Player, Coord, P0, P1, P2 ) %>%
	ggplot +
	aes(x=Time, y=Coord, color=Player) +
#	geom_point() +
	geom_line(alpha=0.4) +
	geom_smooth() +
	facet_wrap(~Notes, scales="free_x") +
	scale_color_colorblind() 
	
th.player = 75
players = res %>%
	mutate(
		C01 = ifelse(P0>th.player & P1>th.player, C01, 0),
		C12 = ifelse(P1>th.player & P2>th.player, C12, 0),
		C20 = ifelse(P2>th.player & P0>th.player, C20, 0)
	) %>%
	na.omit
	
players %>%
	gather( Pair, Coord, C01, C12, C20 ) %>%
	ggplot +
	aes(x=Time, y=Coord, color=Pair) +
#	geom_point() +
	geom_line(alpha=0.4) +
	geom_smooth() +
	facet_wrap(~Notes, scales="free_x") +
	scale_color_colorblind() +
	coord_cartesian(ylim=c(-1,1))

