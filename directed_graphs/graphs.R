# https://www.data-imaginist.com/2017/introducing-tidygraph/

library(ggraph)
library(tidygraph)
library(tidyverse)
require(ggthemes)
require(magrittr)

edge = tribble(
	~Actor,				~Impacts,			~How,
	"Day of Week",		"Lights",			"Pallete",
	"Day of Week",		"Lights",			"Brightness",
	"Day of Week",		"Sound",			"Pallete",
	"Day of Week",		"Sound",			"Volume",
	"Time of Day",		"Lights",			"Enable",
	"Time of Day",		"Mister",			"Enable",
	"Time of Day",		"Sound",			"Pallete",
	"Time of Day",		"Flame/Fog",		"Enable",
	"Temperature",		"Flame/Fog",		"Enable",
	"Temperature",		"Mister",			"Enable",
	"Wind/Dust",		"Mister",			"Enable",
	"Wind/Dust",		"Flame/Fog",		"Enable",
	"Wind/Dust",		"Lights",			"Enable",
	"Trained Operator",	"Flame/Fog",		"Enable",
	"Motion",			"Lights",			"Participation",
	"Motion",			"Sound",			"Participation",
	"Motion",			"Coordination",		"Participation",
	"Coordination",		"Lights",			"Participation",
	"Coordination",		"Sound",			"Participation",
	"Coordination",		"Mister",			"Participation",
	"Coordination",		"Flame/Fog",		"Participation"
)

inputs = tribble(
	~name,				~Category,
	"Day of Week",		"Time",
	"Time of Day",		"Time",
	"Motion",			"Participant",
	"Coordination",		"Participant",
	"Temperature",		"Environment",
	"Wind/Dust",		"Environment",
	"Trained Operator",	"Environment"
)
outputs = edge %>% 
	filter(!Impacts %in% inputs$name) %$% 
	tibble(Category="Output", name=Impacts) %>%
	distinct
node.information = bind_rows(inputs, outputs) %>%
	mutate( Category = factor(Category, levels=c("Output","Participant","Time","Environment")) )
	

gph = as_tbl_graph(edge) %>%
	activate(nodes) %>%
	left_join(node.information) 

gph %>% 
#	ggraph(layout='fr') +
#	ggraph(layout='gem') +
#	ggraph(layout='dh') +
	ggraph(layout='nicely') +
#	ggraph(layout='star', center = 6) +
	geom_edge_link(aes(color=How, linetype=How), arrow=arrow(type="closed", angle=15)) + 
	geom_node_point() + 
    geom_node_label(aes(label = name, color=Category), fill = 'white', repel=T) +
	scale_color_colorblind() +
	scale_fill_colorblind() +
	theme(
		axis.title.x=element_blank(), axis.text.x=element_blank(), axis.ticks.x=element_blank(),
		axis.title.y=element_blank(), axis.text.y=element_blank(), axis.ticks.y=element_blank()
	)
	
ggsave("interaction.png", width=5, height=5, units="in", dpi=300)
	
	

	