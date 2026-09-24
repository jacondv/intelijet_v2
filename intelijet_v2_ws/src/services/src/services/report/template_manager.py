from jinja2 import Environment, FileSystemLoader
import os

TEMPLATE_PATH = os.path.join(os.path.dirname(__file__), 'assets/templates')

def render_template(template_name, data):
    env = Environment(loader=FileSystemLoader(TEMPLATE_PATH))
    template = env.get_template(template_name)
    return template.render(data=data)
