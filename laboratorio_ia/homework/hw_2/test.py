charactersToSearch = charactersToSearch.encode('utf-8')

UNIOnlyAlphabet = charactersToSearch.decode('UTF-8')
query = re.compile('[' + UNIOnlyAlphabet + ']{2,}$', re.U).match

words = set(word.decode('UTF-8').rstrip('\n') for word in open('spanishList.txt') if query(word.decode('UTF-8')))
