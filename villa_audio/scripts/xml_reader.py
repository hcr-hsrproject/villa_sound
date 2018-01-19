"""
This script may be used to parse a RoboCup@Home
XML file a lexicon file for the CKY parser. 
"""

import sys
import xml.etree.ElementTree as ET

class XML_Reader: 
    def __init__(self): 
        pass

    def parse_rooms(self, root): 
        #Will hold a list of rooms. 
        entries = {'type': 'rooms', 'entries': []}

        #Iterate over rooms. 
        for room in root: 
            #Dictionary containing info for room. 
            room_dict = room.attrib

            #Add locations within room. 
            room_dict['locations'] = [location.attrib for location in room]

            #Add room dictionary to list of rooms. 
            entries['entries'].append(room_dict)

        return entries

    def parse_categories(self, root): 
        #Will hold list of object categories. 
        entries = {'type': 'categories', 'entries': []}

        #Iterate over categories. 
        for category in root: 
            #Dictionary containing info for category. 
            category_dict = category.attrib

            #Add objects pertaining to category. 
            category_dict['objects'] = [obj.attrib for obj in category]

            #Add category to list of categories. 
            entries['entries'].append(category_dict)

        return entries

    def parse_names(self, root): 
        #Will hold list of names. 
        entries = {'type': 'names', 'entries': []}

        #Iterate over names. 
        for name in root: 
            #Dictionary containing info for name. 
            name_dict = name.attrib

            #Add actual name to dictionary. 
            name_dict['name'] = name.text

            #Add name to list of names. 
            entries['entries'].append(name_dict)

        return entries

    def parse_questions(self, root):
        #Will hold list of questions. 
        entries = {'type': 'questions', 'entries': []}

        #Iterate over questions. 
        for question in root: 
            #Dictionary containing info for question. 
            question_dict = question.attrib

            #Get question and answer. 
            for element in question: 
                if element.tag == 'q':
                    #Add question. 
                    question_dict['q'] = element.text
                elif element.tag == 'a':
                    #Add answer. #TODO Deal with ... answers. 
                    question_dict['a'] = element.text

            #Add question to list of questions. 
            entries['entries'].append(question_dict)

        return entries

    def parse_gestures(self, root):
        #Will hold list of gestures. 
        entries = {'type': 'gestures', 'entries': []}

        #Iterate over gestures. 
        for gesture in root: 
            #All info is contained within first level of tree, therefore just add it. 
            entries['entries'].append(gesture.attrib)

        return entries

    def parse_xml(self, xml_file): 
        """
        Parses a RoboCup@Home XML file
        and returns its entries in a dictionary. 

        xml_file - The XML file to parse.  
        """
        #Parse XML file and get root node. 
        root = ET.parse(xml_file).getroot()

        #Root must be of a particular known type for RoboCup@Home.
        if root.tag == 'rooms':
            entries = self.parse_rooms(root)
        elif root.tag == 'categories':
            entries = self.parse_categories(root)
        elif root.tag == 'names':
            entries = self.parse_names(root)
        elif root.tag == 'questions':
            entries = self.parse_questions(root)
        elif root.tag == 'gestures':
            entries = self.parse_gestures(root) 
        else:
            raise NotImplementedError('Parsing for root "'+ root.tag + '" has not been implemented!')

        #Return the retrieved entries. 
        return entries

    def get_entries_as_sets(self, entries): 
        #Populate based on type of entry. 
        entry_type = entries['type']
        entries = entries['entries']

        #Will return dict of lists for each subtype. 
        set_dict = {}

        if entry_type == 'rooms':
            
            set_dict['rooms'] = set()
            set_dict['locations'] = set()

            #List contains rooms, which themselves contain locations. 
            for room in entries: 
                #Add room to set. 
                set_dict['rooms'].add(room['name'])

                #Now add all locations within room. 
                for location in room['locations']:
                    set_dict['locations'].add(location['name'])

        elif entry_type == 'names':
           
            set_dict['names'] = set()

            #Entries consists only of names. 
            for name in entries:
                set_dict['names'].add(name['name'])

        elif entry_type == 'gestures':
           
            set_dict['gestures'] = set()

            #Entries consist only of gestures. 
            for gesture in entries: 
                set_dict['gestures'].add(gesture['name'])

        elif entry_type == 'questions':
           
            set_dict['questions'] = set()

            #Entries consists only of questions. 
            for question in entries: 
                set_dict['questions'].add(question['q'])

        elif entry_type == 'categories': 
       
            set_dict['categories'] = set()
            set_dict['objects'] = set()

            #Each category will itself contain objects. 
            for category in entries:
                #Add category to set. 
                set_dict['categories'].add(category['name'])

                #Now add all objects belonging to category. 
                for obj in category['objects']:
                    set_dict['objects'].add(obj['name'])

        else:
            raise ValueError('"' + entry_type + '" is not a valid entry type!')

        #Now return the dictionary of sets. 
        return set_dict

    def xml_to_lex(self, xml_file, lex_file):
        """
        Populate a lexicon file with
        entries from a given xml file. 

        lex_file - The lexicon file to populate. May
                   already have entries. This method
                   only add entries. 

        xml_file - The xml file to read. Should be
                   a file containing Locations, Objects,
                   People, etc. (i.e. categories from RoboCup@Home). 
        """
        #Parse XML file to get entries. 
        entries = self.parse_xml(xml_file)

        #Now add the entries to the lexicon's file. 
        self.add_xml_entries_to_lex_file(entries, lex_file)

    def add_xml_entries_to_lex_file(self, entries, lex_file): 
        """
        Given a dictionary of RoboCup@Home XML entries,
        add them to the lexicon and append the 
        additions to the lexicon's file. 

        entries - A dictionary containing RoboCup@Home
                  entity entries. entries can be 
                  extracted using xml_reader.py. 

        lex_file - The lexicon file to write them to. 
        """
        #First see what type of entries we're getting. 
        entry_type = entries['type']

        if entry_type == 'rooms':
            self.write_room_entries(entries, lex_file)
        elif entry_type == 'categories':
            self.write_category_entries(entries, lex_file)
        elif entry_type == 'names':
            self.write_name_entries(entries, lex_file)
        elif entry_type == 'questions':
            self.write_question_entries(entries, lex_file)
        elif entry_type == 'gestures':
            self.write_gesture_entries(entries, lex_file)
        else:
            raise NotImplementedError('"' + entry_type + '" entry type has not been implemented!')

    def write_room_entries(self, entries, lex_file): 
        assert(entries['type'] == 'rooms')

        #Open lexicon file for writing. 
        lex_file = open(lex_file, 'a')

        #Write first line denoting rooms. 
        lex_file.write('\n#Rooms\n')

        #Fetch list of rooms. 
        rooms = entries['entries']

        for room in rooms:
            #Add entry for room itself. 
            room_name = room['name'].lower()

            #Prepare entry (i.e. name :- CCG : semantic_form)
            entry = room_name + ' :- ' + '(ROOM) : ' + room_name.replace(' ', '_')

            #Write entry to file. 
            lex_file.write(entry + '\n')

            #Add location entries for within the room. 
            for location in room['locations']:
                loc_name = location['name'].lower()
                entry = loc_name + ' :- ' + '(LOC) : ' + loc_name.replace(' ', '_')

                lex_file.write(entry + '\n')

    def write_category_entries(self, entries, lex_file): 
        assert(entries['type'] == 'categories')

        #Open lexicon file for writing. 
        lex_file = open(lex_file, 'a')

        #Write first line denoting categories. 
        lex_file.write('\n#Categories\n')

        #Fetch list of rooms. 
        categories = entries['entries']

        for category in categories:
            #Add entry for category itself. 
            category_name = category['name'].lower()

            #Prepare entry (i.e. name :- CCG : semantic_form)
            entry = category_name + ' :- ' + '(CAT) : ' + category_name.replace(' ', '_')

            #Write entry to file. 
            lex_file.write(entry + '\n')

            #Add entries for objects that fall under category. 
            for obj in category['objects']:
                obj_name = obj['name'].lower()
                entry = obj_name + ' :- ' + '(OBJ) : ' + obj_name.replace(' ', '_')

                lex_file.write(entry + '\n')

    def write_gesture_entries(self, entries, lex_file): 
        assert(entries['type'] == 'gestures')

        #Open lexicon file for writing. 
        lex_file = open(lex_file, 'a')

        #Write first line denoting gestures. 
        lex_file.write('\n#Gestures\n')

        #Fetch list of rooms. 
        gestures = entries['entries']

        for gesture in gestures:
            #Add entry for category itself. 
            gesture_name = gesture['name'].lower()

            #Prepare entry (i.e. name :- CCG : semantic_form)
            entry = gesture_name + ' :- ' + '(GEST) : ' + gesture_name.replace(' ', '_')

            #Write entry to file. 
            lex_file.write(entry + '\n')

    def write_question_entries(self, entries, lex_file): 
        assert(entries['type'] == 'questions')

        #Open lexicon file for writing. 
        lex_file = open(lex_file, 'a')

        #Write first line denoting rooms. 
        lex_file.write('\n#Predefined Questions\n')

        #Fetch list of rooms. 
        questions = entries['entries']

        for question in questions:
            #Get question and answer. 
            q, a = [question['q'].lower(), question['a'].lower()]

            #Prepare entry (i.e. question :- CCG : semantic_form with answer)
            entry = q + ' :- ' + '(QA) : say("' + a + '")'

            #Write entry to file. 
            lex_file.write(entry + '\n')

def print_usage():
    print('To populate lexicon file with entries from XML: python3 xml_reader.py xml_to_lex [xml_file] [lexicon_file]')

if __name__ == '__main__':
    if len(sys.argv) < 2: 
        print_usage()
    elif sys.argv[1] == 'xml_to_lex':
        reader = XML_Reader()
        reader.xml_to_lex(sys.argv[2], sys.argv[3])
    else:
        print_usage()
