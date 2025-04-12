# ai_core.py

from openai import OpenAI
import os
import json

##########################################################################

def get_api_key():
    api_key = os.environ.get('OPENAI_API_KEY')
    if not api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set")
    return api_key

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