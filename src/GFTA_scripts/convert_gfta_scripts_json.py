#!/usr/bin/python
import json
import ast

docs_to_change = ["narrative_script"]

for doc_name in docs_to_change:
	lines = open(doc_name + '.txt').readlines()
	outfile = open(doc_name + '.json', 'w')

	for line in lines:
		line = line.strip() #remove newlines
		prompt_list = line.split('\t')

		new_prompt_list = []

		for prompt in prompt_list:
			if len(prompt) > 0:
				print("OLD PROMPT")
				print(prompt)
				prompt = prompt[1:-1].split(',')

				anim = prompt[0]
				audio =  prompt[1]
				tts = ''
				label = prompt[2]

				new_prompt = [anim.strip(), audio.strip(), tts.strip(), label.strip()]
				#print(prompt)
				#print(new_prompt)
				new_prompt_list.append(new_prompt)

		
		outfile.write(json.dumps(new_prompt_list))
		outfile.write('\n')
	outfile.close()


	#for line in lines:
	#	print(line)



