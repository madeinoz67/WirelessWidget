/*
 * Copyright (c) 2007, Swedish Institute of Computer Science. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution. 3. Neither the name of the
 * Institute nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: AbstractApplicationMoteType.java,v 1.3 2009/03/09 15:38:10 fros4943 Exp $
 */

package se.sics.cooja.motes;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.io.File;
import java.util.*;

import javax.swing.*;
import org.apache.log4j.Logger;
import org.jdom.Element;

import se.sics.cooja.*;
import se.sics.cooja.interfaces.ApplicationRadio;
import se.sics.cooja.interfaces.Position;

@ClassDescription("Application Mote Type")
public abstract class AbstractApplicationMoteType implements MoteType {
  private static Logger logger = Logger.getLogger(AbstractApplicationMoteType.class);

  // Mote type specific data
  private String identifier = null;

  private String description = null;

  private Class<? extends MoteInterface>[] moteInterfaces = null;

  // Type specific class configuration
  private ProjectConfig myConfig = null;

  public AbstractApplicationMoteType() {
  }

  public AbstractApplicationMoteType(String identifier) {
    this.identifier = identifier;
    description = "Application Mote Type #" + identifier;
  }

  public boolean configureAndInit(Container parentContainer, Simulation simulation, boolean visAvailable) {

    if (identifier == null) {
      // Create unique identifier
      int counter = 0;
      boolean identifierOK = false;
      while (!identifierOK) {
        counter++;
        identifier = "apptype" + counter;
        identifierOK = true;

        // Check if identifier is already used by some other type
        for (MoteType existingMoteType : simulation.getMoteTypes()) {
          if (existingMoteType != this
              && existingMoteType.getIdentifier().equals(identifier)) {
            identifierOK = false;
            break;
          }
        }
      }
    }

    if (description == null) {
      // Create description
      description = "Application Mote Type #" + identifier;
    }

    moteInterfaces = new Class[2];
    moteInterfaces[0] = Position.class;
    moteInterfaces[1] = ApplicationRadio.class;

    return true;
  }

  public String getIdentifier() {
    return identifier;
  }

  public void setIdentifier(String identifier) {
    this.identifier = identifier;
  }

  public String getDescription() {
    return description;
  }

  public void setDescription(String description) {
    this.description = description;
  }

  public Class<? extends MoteInterface>[] getMoteInterfaceClasses() {
    return moteInterfaces;
  }

  public void setMoteInterfaceClasses(Class<? extends MoteInterface>[] moteInterfaces) {
    this.moteInterfaces = moteInterfaces;
  }

  public JPanel getTypeVisualizer() {
    JPanel panel = new JPanel();
    JLabel label = new JLabel();
    JPanel smallPane;

    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

    // Identifier
    smallPane = new JPanel(new BorderLayout());
    label = new JLabel("Identifier");
    smallPane.add(BorderLayout.WEST, label);
    label = new JLabel(identifier);
    smallPane.add(BorderLayout.EAST, label);
    panel.add(smallPane);

    // Description
    smallPane = new JPanel(new BorderLayout());
    label = new JLabel("Description");
    smallPane.add(BorderLayout.WEST, label);
    label = new JLabel(description);
    smallPane.add(BorderLayout.EAST, label);
    panel.add(smallPane);

    // Mote Interfaces
    smallPane = new JPanel(new BorderLayout());
    label = new JLabel("Mote interfaces");
    smallPane.add(BorderLayout.WEST, label);
    panel.add(smallPane);

    for (Class moteInterface : moteInterfaces) {
      smallPane = new JPanel(new BorderLayout());
      label = new JLabel(moteInterface.getSimpleName());
      smallPane.add(BorderLayout.EAST, label);
      panel.add(smallPane);
    }

    panel.add(Box.createRigidArea(new Dimension(0, 5)));
    return panel;
  }

  public File getContikiSourceFile() {
    return null; /* Contiki-independent */
  }

  public File getContikiFirmwareFile() {
    return null; /* Contiki-independent */
  }

  public void setContikiSourceFile(File file) {
    /* Contiki-independent */
  }

  public void setContikiFirmwareFile(File file) {
    /* Contiki-independent */
  }

  public String getCompileCommands() {
    /* Contiki-independent */
    return null;
  }

  public void setCompileCommands(String commands) {
    /* Contiki-independent */
  }

  public ProjectConfig getConfig() {
    return myConfig;
  }

  public Collection<Element> getConfigXML() {
    Vector<Element> config = new Vector<Element>();

    Element element;

    // Identifier
    element = new Element("identifier");
    element.setText(getIdentifier());
    config.add(element);

    // Description
    element = new Element("description");
    element.setText(getDescription());
    config.add(element);

    // Mote interfaces
    for (Class moteInterface : getMoteInterfaceClasses()) {
      element = new Element("moteinterface");
      element.setText(moteInterface.getName());
      config.add(element);
    }

    return config;
  }

  public boolean setConfigXML(Simulation simulation,
      Collection<Element> configXML, boolean visAvailable) {
    ArrayList<Class<? extends MoteInterface>> moteInterfacesList = new ArrayList<Class<? extends MoteInterface>>();
    for (Element element : configXML) {

      String name = element.getName();

      if (name.equals("identifier")) {
        identifier = element.getText();
      } else if (name.equals("description")) {
        description = element.getText();
      } else if (name.equals("moteinterface")) {
        Class<? extends MoteInterface> moteInterfaceClass =
          simulation.getGUI().tryLoadClass(
              this, MoteInterface.class, element.getText().trim());

        if (moteInterfaceClass == null) {
          logger.warn("Can't find mote interface class: " + element.getText());
        } else {
          moteInterfacesList.add(moteInterfaceClass);
        }
      } else {
        logger.fatal("Unrecognized entry in loaded configuration: " + name);
      }
    }

    moteInterfaces = new Class[moteInterfacesList.size()];
    moteInterfacesList.toArray(moteInterfaces);

    boolean createdOK = configureAndInit(GUI.getTopParentContainer(), simulation, visAvailable);
    return createdOK;
  }

}
