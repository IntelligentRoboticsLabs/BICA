#!/usr/bin/python

import rospy
from rospkg import RosPack
from std_srvs.srv import *
import fnmatch
import os

class PddlBuilder:

    def __init__(self, template, domain, pddl_out):
        self.template = template
        self.domain = domain
        self.pddl_out = pddl_out

        self.trigger_srv = rospy.Service('~update_domain', Trigger, self.update_domain)

    def update_domain(self, req):
        self.update_pddl_file()

        return TriggerResponse(True, "")

    def get_pddl_files_in_ws(self, pddl_dirs, name):
        files = []
        for dir in pddl_dirs:
            for root, dirnames, filenames in os.walk(dir):
                for filename in fnmatch.filter(filenames, name):
                    files.append(os.path.join(root, filename))
        return files

    def get_pddl_pkgs(self):
        dirs = []
        return dirs

    def get_file_content(self, files):
        content = ""
        for file_ in files:

            #content += "\n;; " + file_ + " ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n\n"
            content += open(file_, "r").read()
            content += "\n"
        return content

    def remove_duplicated(self, in_content):
        set_of_lines = set(in_content.splitlines())
        return '\n'.join(set_of_lines)

    def update_pddl_file(self):

        self.clean_pddl()

        types       = ""
        predicates  = ""
        actions     = ""


        pddl_dirs = []
        if rospy.has_param('~pddl_packages'):
            pddl_pkgs = rospy.get_param('~pddl_packages')

            if isinstance(pddl_pkgs, (list,)):
                for pkg in pddl_pkgs:
                    try:
                        pkg_path = rospack.get_path(pkg)
                        pddl_dirs.append(pkg_path)
                    except:
                        rospy.logerror("No existe package[" + pkg + "]")
                        pass
            elif isinstance(pddl_pkgs, basestring):
                try:
                    pkg_path = rospack.get_path(pddl_pkgs)
                    pddl_dirs.append(pkg_path)
                except:
                    rospy.logerror("No existe package[" + pddl_pkgs + "]")
                    pass
            else:
                rospy.logerror("[pddl_builder] Unable to process: " + pddl_pkgs)

            rospy.loginfo("[pddl_builder] set pddl_dirs to:")
            rospy.loginfo(pddl_dirs)
        else:
            rospy.logwarn("[pddl_builder] set pddl_pkgs to all")
            pddl_dirs.append(os.environ['ROS_PACKAGE_PATH'].split(':')[0])

        type_files = self.get_pddl_files_in_ws(pddl_dirs, "types.pddl")
        predicate_files = self.get_pddl_files_in_ws(pddl_dirs, "predicates.pddl")
        action_files =  self.get_pddl_files_in_ws(pddl_dirs, "actions.pddl")
        function_files =  self.get_pddl_files_in_ws(pddl_dirs, "functions.pddl")

        types_content = self.get_file_content(type_files)
        predicates_content = self.get_file_content(predicate_files)
        functions_content = self.get_file_content(function_files)
        actions_content = self.get_file_content(action_files)
        template_content = self.get_file_content([self.template])

        types_content = self.remove_duplicated(types_content)
        predicates_content = self.remove_duplicated(predicates_content)

        template_content = template_content.replace("$$TYPES$$", types_content)
        template_content = template_content.replace("$$PREDICATES$$", predicates_content)
        template_content = template_content.replace("$$FUNCTIONS$$", functions_content)
        template_content = template_content.replace("$$ACTIONS$$", actions_content)
        template_content = template_content.replace("$$DOMAIN$$", self.domain)


        try:
            with open(self.pddl_out, "w") as file:
                file.write(template_content)
        except IOError as e:
            rospy.logerr("I/O error %s returned the invalid value %s", e.errno, e.strerror)
            return

    def clean_pddl(self):
        os.system("rm " + self.pddl_out+ " > /dev/null 2> /dev/null")

if __name__ == "__main__":

    try:
        rospy.init_node('pddl_builder', anonymous=False)
        rospack = RosPack()

        if rospy.has_param("~pddl_domain_file"):
            pddl_out_file = rospy.get_param('~pddl_domain_file')
            rospy.loginfo("[pddl_builder] set pddl_out_file to ["+pddl_out_file+"]")
        else:
            rospy.logwarn("[pddl_builder] set pddl_out_file to default")
            pddl_out_file = "/tmp/domain.pddl"

        if rospy.has_param("~pddl_domain"):
            domain = rospy.get_param('~pddl_domain')
            rospy.loginfo("[pddl_builder] set domain to ["+domain+"]")
        else:
            rospy.logwarn("[pddl_builder] set pddl_domain to default")
            domain = "default_domain"


        pddl_template = rospack.get_path('bica_planning') + "/pddl/template.pddl"

        builder = PddlBuilder(pddl_template, domain, pddl_out_file)
        builder.update_pddl_file()

        rospy.spin()

        builder.clean_pddl()

    except rospy.ROSInterruptException:
        pass
