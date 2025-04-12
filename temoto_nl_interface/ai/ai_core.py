# ai_core.py

from openai import OpenAI
import os
import json

####################################### Update filepath for api key ######

file_path = '~/CHATGPT_KEY'

##########################################################################

# Function to retrieve API key
def get_api_key():
    expanded_file_path = os.path.expanduser(file_path)
    with open(expanded_file_path, 'r') as file:
        return file.read().strip()

    # TEMPERATURE       - Controls response creativity
    # MAX_TOKENS        - Maximum tokens in response
    # FREQUENCY_PENALTY - Discourages repetition
    # PRESENCE_PENALTY  - Encourages topic introduction

# Model for handling text
def AI_Language_Prompt(messages, TEMPERATURE, MAX_TOKENS, FREQUENCY_PENALTY, PRESENCE_PENALTY):

    try: 
        api_key = get_api_key()
        client = OpenAI(api_key=api_key)

        completion = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=TEMPERATURE,
            max_tokens=MAX_TOKENS,
            top_p=1,
            frequency_penalty=FREQUENCY_PENALTY,
            presence_penalty=PRESENCE_PENALTY,
        )

        response = completion.choices[0].message.content.strip()
        return response

    except Exception as e:
        response = json.dumps({
                    "success": "false",
                    "message": f"Error sending data to LLM: {e}"
                })
        return response


# Model for handling images
def AI_Image_Prompt(messages, TEMPERATURE, MAX_TOKENS, FREQUENCY_PENALTY, PRESENCE_PENALTY):

    try: 
        api_key = get_api_key()
        client = OpenAI(api_key=api_key)

        completion = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=TEMPERATURE,
            max_tokens=MAX_TOKENS,
            top_p=1,
            frequency_penalty=FREQUENCY_PENALTY,
            presence_penalty=PRESENCE_PENALTY,
        )
        response = completion.choices[0].message.content.strip()
        return response

    except Exception as e:
        response = json.dumps({
                    "success": "false",
                    "message": f"Error sending data to LLM: {e}"
                })
        return response