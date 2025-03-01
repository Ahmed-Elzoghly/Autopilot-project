import shutil

# Create a zip file for the Simulink template (since I cannot generate real .slx files directly)
template_folder = "/mnt/data/Simulink_Template"
shutil.make_archive(template_folder, 'zip', template_folder)

# Output path for the user to download
"/mnt/data/Simulink_Template.zip"
