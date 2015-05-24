ifeq ($(RTAI),Y)
  RTAI := y
endif
ifeq ($(XENO),Y)
  XENO := y
endif


ifeq ($(RTAI),y)
ifeq ($(XENO),y)
    $(error build with Xenomai and RTAI at the same time is impossible, unset one variable)
endif
endif

ifeq ($(RTAI),y)
  rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
    $(shell rtai-config --prefix)/bin/rtai-config,\
    $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)
endif

ifeq ($(XENO),y)
  xenoconfigbin := $(if $(@shell xeno-config --prefix),\
    $(shell xeno-config --prefix)/bin/xeno-config,\
    $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)
endif

default: all

checks_libraries:
ifeq ($(RTAI),y)
ifeq ($(rtaiconfigbin),)
	$(warning Dont found RTAI. Please be sure you install RTAI on your pc or)
	$(warning add <rtai-install-path>/bin to your PATH variable.)
	$(error !!!!! or disable RTAI !!!!!)
else
	@echo "build with RTAI selected"
endif
endif
ifeq ($(XENO),y)
ifeq ($(xenoconfigbin),)
	$(warning Dont found Xenomai. Please be sure you install Xenomai on your pc or)
	$(warning add <xeno-install-path>/bin to your PATH variable.)
	$(error !!!!! or disable XENO !!!!!)
else
	@echo "build with Xenomai selected"
endif
endif

all: checks_libraries
	@make -C driver/linux
	@make -C pniolib
	@make -C layer2lib
	@make -C servlib

test: checks_libraries
	@make -C testapps
	@make -C examples

clean:
	@make -C driver/linux clean
	@make -C pniolib clean
	@make -C layer2lib clean
	@make -C servlib clean
	@make -C testapps clean
	@make -C examples clean

install: checks_libraries
	@make -C driver/linux install
	@make -C pniolib install
	@make -C layer2lib install
	@make -C servlib install
	@mkdir --mode=777 -p /etc/16xx_config/backup

test_install: checks_libraries
	@make -C testapps install

deinstall:
	@make -C driver/linux deinstall
	@make -C pniolib deinstall
	@make -C layer2lib deinstall
	@make -C servlib deinstall
	@rm -r -f /etc/16xx_config

test_deinstall:
	@make -C testapps deinstall

lint:
	@make -C pniolib lint
	@make -C testapps lint

indent:
	@indent -hnl -l90 -lc90 -nbad -nbap -nbc -nbfda \
	 -nsc -nss -ncs -nut -brs -br -bs -bbo -cdw -ce \
	 -i4 -npcs -nprs -npsl -nsaf -nsai -nsaw -nlp -ci4 \
	 -ip0 -nsc -nsfa $(shell find . -type f -name *.c)
	@indent -hnl -l90 -lc90 -nbad -nbap -nbc -nbfda \
	 -nsc -nss -ncs -nut -brs -br -bs -bbo -cdw -ce \
	 -i4 -npcs -nprs -npsl -nsaf -nsai -nsaw -nlp -ci4 \
	 -ip0 -nsc -nsfa $(shell find . -type f -name *.cpp)
	@indent -hnl -l90 -lc90 -nbad -nbap -nbc -nbfda \
	 -nsc -nss -ncs -nut -brs -br -bs -bbo -cdw -ce \
	 -i4 -npcs -nprs -npsl -nsaf -nsai -nsaw -nlp -ci4 \
	 -ip0 -nsc -nsfa $(shell find . -type f -name *.h)

load:
	@chmod 0755 ./driver/linux/cp16xxloader
	@./driver/linux/cp16xxloader start

unload:
	@chmod 0755 ./driver/linux/cp16xxloader
	@./driver/linux/cp16xxloader stop

reload:
	@chmod 0755 ./driver/linux/cp16xxloader
	@./driver/linux/cp16xxloader restart

show_warning:
	$(warning Tested only with different SuSE flawours from 9.2 up to 10.2)
	$(warning can work with any LSB Linux distribution, but no warranty is given)

autoload: show_warning
	@chmod 0755 ./driver/linux/cp16xxloader
	@cp ./driver/linux/cp16xxloader /etc/init.d
	@/sbin/insserv -d /etc/init.d/cp16xxloader
	@./driver/linux/cp16xxloader start

removeautoload: show_warning
	@chmod 0755 ./driver/linux/cp16xxloader
	@/sbin/insserv -rf /etc/init.d/cp16xxloader
	@rm -f /etc/init.d/cp16xxloader
	@./driver/linux/cp16xxloader stop

cpreset: test
	@make -C testapps cpreset

DATE=$(shell date +%Y%m%d)
CURRENT=$(shell basename $(PWD))

distclean: clean
	#restore directories and files permissions
	#do it first, becose bash caches file list in first place
	#and chmod complains about missing files
	@chmod -Rf 0755 $(shell find . -type d -print0 | xargs -0)
	@chmod -Rf 0644 $(shell find . -type f -print0 | xargs -0)
	@chmod -f 0755 ./driver/linux/cp16xxloader
	#remove backup files
	@rm -f $(shell find . -type f -name '*~' -print0 | xargs -0)
	#remove hidden files and directories
	@rm -rf $(shell find . -name '.+' -print0 | xargs -0)
	#remove unimportant MS VisualStudio files
	@rm -f $(shell find . -name '*.ncb' -print0 | xargs -0)
	@rm -f $(shell find . -name '*.suo' -print0 | xargs -0)
	@rm -f $(shell find . -name '*.user' -print0 | xargs -0)
	@rm -f $(shell find . -name 'BuildLog.htm' -print0 | xargs -0)
	#remove white space before EOL
	@perl -i -p -e 's/[\t ]+$$//g' $(shell find . -type f -name '*.c' -print0 | xargs -0)
	@perl -i -p -e 's/[\t ]+$$//g' $(shell find . -type f -name '*.cpp' -print0 | xargs -0)
	@perl -i -p -e 's/[\t ]+$$//g' $(shell find . -type f -name '*.h' -print0 | xargs -0)

archiv: clean distclean
	@tar -C .. -cvz -f ../host-$(DATE).tar.gz $(CURRENT) > /dev/null
	@echo 'Created an archiv host-$(DATE).tar.gz'

dewin:
	# restore UNIX end of line
	dos2unix $(shell find . -type f | grep -v doc)
	# remove white space at EOL
	perl -i -p -e 's/[\t ]+$$//g' $(shell find . -type f | grep -v zip)
	# restore directories and files permissions
	chmod -R 0755 $(shell find . -type d)
	chmod -R 0644 $(shell find . -type f | grep -v doc)
