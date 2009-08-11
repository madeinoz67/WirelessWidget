/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: Mote2MoteRelations.java,v 1.1 2009/02/26 13:31:25 fros4943 Exp $
 */

package se.sics.cooja.interfaces;

import java.util.*;

import javax.swing.*;
import org.apache.log4j.Logger;
import org.jdom.Element;

import se.sics.cooja.*;

/**
 * Mote2Mote Relations is used to show mote relations in simulated
 * applications, typically for debugging or visualization purposes.
 *
 * The interface is write-only: the simulated Contiki has no knowledge of current relations
 * with other motes. The Contiki application can, however, add and remove relations.
 *
 * A Contiki application adds/removes a relation by outputting a simple messages on its log interface,
 * typically via printf()'s of the serial port.
 *
 * Syntax:
 * "<relation identifier #L> <destination mote ID> <add/remove>"
 *
 * Example, add relation between this mote and mote with ID 1
 * "#L 1 1"
 *
 * Example, remove relation between this mote and mote with ID 1
 * "#L 1 0"
 *
 * Example, remove relation between this mote and mote with ID 2
 * "#L 2 0"
 *
 * @author Fredrik Osterlind
 */
@ClassDescription("Mote2Mote Relations")
public class Mote2MoteRelations extends MoteInterface {
  private static Logger logger = Logger.getLogger(Mote2MoteRelations.class);
  private Mote mote = null;

  private Log log;
  private ArrayList<Mote> relations = new ArrayList<Mote>();
  private GUI gui;

  public Mote2MoteRelations(Mote mote) {
    this.mote = mote;
    this.gui = mote.getSimulation().getGUI();

    /* XXX Bug: Infinitely repeating runnable if no log interface exists! */
    Runnable init = new Runnable() {
      public void run() {
        if (Mote2MoteRelations.this.mote.getInterfaces() == null
            || Mote2MoteRelations.this.mote.getInterfaces().getLog() == null) {
          SwingUtilities.invokeLater(this);
          return;
        }

        log = Mote2MoteRelations.this.mote.getInterfaces().getLog();
        if (log == null) {
          SwingUtilities.invokeLater(this);
          return;
        }

        log.addObserver(logObserver);
      }
    };
    init.run();
  }

  private Observer logObserver = new Observer() {
    public void update(Observable o, Object arg) {
      String msg = log.getLastLogMessage();
      if (msg == null) {
        return;
      }

      if (!msg.startsWith("#L ")) {
        return;
      }

      String[] args = msg.split(" ");
      if (args.length != 3) {
        return;
      }

      String id = args[1];
      String state = args[2];

      /* Locate destination mote */
      /* TODO Use Rime address interface instead of mote ID? */
      Mote destinationMote = null;
      Mote[] allMotes = Mote2MoteRelations.this.mote.getSimulation().getMotes();
      for (Mote m: allMotes) {
        if (m.getInterfaces().getMoteID() == null) {
          continue;
        }

        if (id.equals("" + m.getInterfaces().getMoteID().getMoteID())) {
          destinationMote = m;
          break;
        }
      }
      if (destinationMote == null) {
        logger.warn("No destination mote with ID: " + id);
        return;
      }
      if (destinationMote == mote) {
        /*logger.warn("Cannot create relation with ourselves");*/
        return;
      }

      /* Change line state */
      if (state.equals("1")) {
        if (relations.contains(destinationMote)) {
          return;
        }
        relations.add(destinationMote);
        gui.addMoteRelation(mote, destinationMote);
      } else {
        relations.remove(destinationMote);
        gui.removeMoteRelation(mote, destinationMote);
      }

      setChanged();
      notifyObservers();
    }
  };

  public JPanel getInterfaceVisualizer() {
    JPanel panel = new JPanel();
    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

    final JLabel countLabel = new JLabel();
    countLabel.setText("Mote has " + relations.size() + " mote relations");
    panel.add(countLabel);

    Observer observer;
    this.addObserver(observer = new Observer() {
      public void update(Observable obs, Object obj) {
        countLabel.setText("Mote has " + relations.size() + " mote relations");
      }
    });

    // Saving observer reference for releaseInterfaceVisualizer
    panel.putClientProperty("intf_obs", observer);

    return panel;
  }

  public void releaseInterfaceVisualizer(JPanel panel) {
    Observer observer = (Observer) panel.getClientProperty("intf_obs");
    if (observer == null) {
      logger.fatal("Error when releasing panel, observer is null");
      return;
    }
    this.deleteObserver(observer);
  }

  public double energyConsumption() {
    return 0;
  }

  public Collection<Element> getConfigXML() {
    return null;
  }

  public void setConfigXML(Collection<Element> configXML, boolean visAvailable) {
  }

}
