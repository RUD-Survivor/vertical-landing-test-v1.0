import sys

try:
    with open('build_err.txt', 'rb') as f:
        content = f.read().decode('gbk', 'replace')
    
    with open('cleaned_errors.txt', 'w', encoding='utf-8') as f:
        for line in content.split('\n'):
            if 'error' in line.lower() or 'fatal' in line.lower() or '.cpp' in line.lower():
                f.write(line + '\n')
    print("Errors extracted to cleaned_errors.txt")
except Exception as e:
    print(f"Error: {e}")
